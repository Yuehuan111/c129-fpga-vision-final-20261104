// FPGA-only projective/OpenCV-fisheye map generation. Verilog-2005.
// Kout^-1 -> legacy roll -> normalized inverse H -> divide -> fisheye -> tangential -> Ksrc.
// H is row-major signed Q16.16; low word is H00. All 27 words latch at start.
// FPGA builds a 4096-segment scale(r^2) LUT and a 1024-entry reciprocal seed LUT.
// No host-generated coordinates, floating point, initial ROM or $readmemh.
// Stream: one coordinate/clock after setup, with a globally stalled pipeline.
// Per-pixel domain/visibility failures produce entry=0; configuration/arithmetic
// setup failures assert error. No coordinate is clipped to hide a black border.
// Numerical domain: max(abs(H)) in [0.5,8], Z >= 1/64 (Q16.32 rays),
// abs(X/Z),abs(Y/Z)<=4, r^2<32; finite positive source focal <=4096 px.
module complete_fisheye_mapper #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,
 parameter signed[31:0] TANGENTIAL_P1_Q16_16=-32'sd13,
 parameter signed[31:0] TANGENTIAL_P2_Q16_16=32'sd18
)(
 input wire clk,rst_n,start,
 input wire signed[31:0] cfg_fx_src,cfg_fy_src,cfg_cx_src,cfg_cy_src,
 input wire signed[31:0] cfg_cx_out,cfg_cy_out,cfg_fx_out,cfg_fy_out,
 input wire signed[31:0] cfg_k1,cfg_k2,cfg_k3,cfg_k4,cfg_cos,cfg_sin,
 input wire[31:0] cfg_mode,input wire[287:0] cfg_projective,
 input wire map_ready,output wire busy,
 output reg done,output wire map_valid,output wire[38:0] map_entry,
 output wire map_last,output wire map_domain_error,output reg error
);
 localparam IDLE=0,INVX=1,INVXW=2,INVY=3,INVYW=4,COMPOSE=5,MULWAIT=6,
  RECIP=7,RECIPW=8,RADSTART=9,SQRTW=10,ATANSTART=11,ATANW=12,
  RADMUL=13,RADDIV=14,RADDIVW=15,RADSTORE=16,RUN=17,PREPARE_RUN=18,
  COMPOSE_CHECK=19;
 localparam signed[47:0] ONE28=48'sd268435456;
 localparam signed[63:0] LIMIT28=64'sd1073741824;
 localparam signed[63:0] MAXX=(IMG_WIDTH-1)*64'sd268435456;
 localparam signed[63:0] MAXY=(IMG_HEIGHT-1)*64'sd268435456;
 // The production constants are the C joint candidate. The zero generate
 // branch retains the original mapper arithmetic AND cycle alignment exactly.
 // Deliberately reject other constants rather than silently using this fixed
 // shift/add implementation for an unrelated calibration.
 localparam TANGENTIAL_ENABLED=TANGENTIAL_P1_Q16_16!=0||TANGENTIAL_P2_Q16_16!=0;
 localparam TANGENTIAL_LATENCY=TANGENTIAL_ENABLED?8:0;
 localparam SOURCE_STAGE=33+TANGENTIAL_LATENCY;
 localparam FINAL_STAGE=35+TANGENTIAL_LATENCY;
 generate if(TANGENTIAL_ENABLED&&
  (TANGENTIAL_P1_Q16_16!=-32'sd13||TANGENTIAL_P2_Q16_16!=32'sd18)) begin: unsupported_tangential_constants
  COMPLETE_TANGENTIAL_PARAMETERS_MUST_BE_0_0_OR_MINUS13_PLUS18 invalid_parameters();
 end endgenerate
 reg[4:0] state,return_state;
 reg[2:0] mwait;reg[5:0] mshift;
 (* syn_preserve=1 *) reg signed[47:0] ma,mb;
 reg signed[47:0] mresult;wire signed[95:0] mp;
 fisheye_multiplier48 setup_multiplier(.clk(clk),.a(ma),.b(mb),.product(mp));
 reg signed[31:0] fx,fy,cx,cy,ox,oy,ofx,ofy,ca,sa;
 reg signed[31:0] h[0:8];reg signed[47:0] k[0:3];
 wire[8:0] matrix_scale_flags;
 // Register each start-time check before the state decision; never reread cfg in INVX.
 (* syn_preserve=1 *) reg[19:0] start_invalid_flags;
 (* syn_preserve=1 *) reg[8:0] start_matrix_scale_flags;
 wire start_configuration_bad=(|start_invalid_flags)||!(|start_matrix_scale_flags);
 genvar hg;generate for(hg=0;hg<9;hg=hg+1)begin: matrix_scale_test
  assign matrix_scale_flags[hg]=$signed(cfg_projective[hg*32+:32])>=32768||$signed(cfg_projective[hg*32+:32])<= -32768;
 end endgenerate
 reg signed[47:0] invfx,invfy,dx[0:2],dy[0:2],origin[0:2];
 reg signed[47:0] temporary,ux,vy,xc,yc;
 reg[1:0] row;reg[4:0] compose_step;
 reg identity,configuration_bad,issued_all;
 reg seed_ready,lut_ready,reuse_lut;
 reg signed[31:0] cached_k[0:3];
 reg[15:0] px,py;
 reg signed[47:0] rayx,rayy,rayz,rowx,rowy,rowz;
 wire signed[48:0] nextx={rayx[47],rayx}+{dx[0][47],dx[0]};
 wire signed[48:0] nexty={rayy[47],rayy}+{dx[1][47],dx[1]};
 wire signed[48:0] nextz={rayz[47],rayz}+{dx[2][47],dx[2]};
 wire signed[48:0] nextrowx={rowx[47],rowx}+{dy[0][47],dy[0]};
 wire signed[48:0] nextrowy={rowy[47],rowy}+{dy[1][47],dy[1]};
 wire signed[48:0] nextrowz={rowz[47],rowz}+{dy[2][47],dy[2]};
 reg ray_bad,row_bad;
 reg[10:0] seed_index;reg[12:0] radial_index;reg[3:0] radial_step;
 reg[31:0] radius;reg signed[47:0] theta,theta2,horner,theta_d;
 reg[32:0] radial_word;
 (* syn_ramstyle="block_ram" *) reg[32:0] radial_lo[0:4095];
 (* syn_ramstyle="block_ram" *) reg[32:0] radial_hi[0:4095];
 (* syn_ramstyle="block_ram" *) reg[31:0] reciprocal[0:1023];
 wire div_done,div_error;wire[63:0] quotient;
 wire div_start=(state==INVX&&!start_configuration_bad)||state==INVY||state==RECIP||state==RADDIV;
 wire[63:0] numerator=state==RECIP?64'd2199023255552:
  state==RADDIV?({32'b0,theta_d[31:0]}<<28):64'h0100000000000000;
 wire[31:0] denominator=state==RECIP?(32'd2049+{20'b0,seed_index,1'b0}):
  state==INVX?ofx:state==INVY?ofy:radius;
 fisheye_unsigned_divider divider(.clk(clk),.rst_n(rst_n),.start(div_start),
  .numerator(numerator),.denominator(denominator),.busy(),.done(div_done),.error(div_error),.quotient(quotient));
 wire sqrt_done,atan_done;wire[31:0] sqrt_root;wire signed[31:0] atan_theta;
 wire[63:0] radicand={2'b0,radial_index,49'b0};
 fisheye_unsigned_sqrt root_unit(.clk(clk),.rst_n(rst_n),.start(state==RADSTART&&radial_index!=0),
  .radicand(radicand),.busy(),.done(sqrt_done),.root(sqrt_root));
 fisheye_atan atan_unit(.clk(clk),.rst_n(rst_n),.start(state==ATANSTART),
  .radius_q28(radius),.busy(),.done(atan_done),.theta_q30(atan_theta));

 // Shared valid/last/invalid shift registers are the only reset pipeline state.
 reg[FINAL_STAGE:0] pv,pl,pbad,pidentity;
 wire advance=!pv[FINAL_STAGE]||map_ready;
 wire issue=state==RUN&&!issued_all;
 wire pipe_advance=advance&&(state==RUN||(|pv));
 assign busy=rst_n&&state!=IDLE;
 assign map_valid=pv[FINAL_STAGE];assign map_last=pl[FINAL_STAGE];
 // The existing invalid flag follows the same global enable as entry35.
 // Expose its cause without changing any coordinate, truncation or latency.
 assign map_domain_error=pbad[FINAL_STAGE]&&!pidentity[FINAL_STAGE];
 reg[38:0] entry34,entry35;assign map_entry=entry35;
 reg[37:0] identity_pipe[0:FINAL_STAGE-1];
 reg signed[47:0] rx0,ry0,rz0;reg[5:0] lead0;
 reg signed[31:0] xp[1:28],yp[1:28];
 reg signed[31:0] mantissa1,mantissa2,seedpipe[2:6],correction6,reciprocal10;
 reg[63:0] radius2_18;
 reg[32:0] lower19,upper19;reg[19:0] fraction19;
 (* syn_preserve=1 *) reg signed[31:0] difference20;
 reg signed[31:0] fraction20,lowerpipe[20:24],scale24;
 reg signed[63:0] us32,vs32,source33u,source33v;
 wire signed[63:0] rounded_u=source33u+64'sd524288;
 wire signed[63:0] rounded_v=source33v+64'sd524288;
 wire signed[63:0] mr_product,nr_product,projectx_product,projecty_product;
 wire signed[63:0] squarex_product,squarey_product,interpolation_product;
 wire signed[63:0] distortx_product,distorty_product,focalx_product,focaly_product;
 wire signed[31:0] focal_input_x,focal_input_y;
 wire tangential_bad;
 generate if(TANGENTIAL_ENABLED) begin: tangential_enabled
  complete_tangential_q28 tangential(.clk(clk),.ce(pipe_advance),
   .x_q28(xp[28]),.y_q28(yp[28]),.x_out_q28(focal_input_x),
   .y_out_q28(focal_input_y),.overflow(tangential_bad));
 end else begin: tangential_zero
  assign focal_input_x=xp[28];assign focal_input_y=yp[28];
  assign tangential_bad=1'b0;
 end endgenerate
 complete_geometry_mul32 m0(clk,pipe_advance,mantissa2,seedpipe[2],mr_product);
 complete_geometry_mul32 m1(clk,pipe_advance,seedpipe[6],correction6,nr_product);
 complete_geometry_mul32 m2(clk,pipe_advance,xp[10],reciprocal10,projectx_product);
 complete_geometry_mul32 m3(clk,pipe_advance,yp[10],reciprocal10,projecty_product);
 complete_geometry_mul32 m4(clk,pipe_advance,xp[14],xp[14],squarex_product);
 complete_geometry_mul32 m5(clk,pipe_advance,yp[14],yp[14],squarey_product);
 complete_geometry_mul32 m6(clk,pipe_advance,difference20,fraction20,interpolation_product);
 complete_geometry_mul32 m7(clk,pipe_advance,xp[24],scale24,distortx_product);
 complete_geometry_mul32 m8(clk,pipe_advance,yp[24],scale24,distorty_product);
 complete_geometry_mul32 m9(clk,pipe_advance,focal_input_x,fx,focalx_product);
 complete_geometry_mul32 m10(clk,pipe_advance,focal_input_y,fy,focaly_product);
 wire signed[63:0] projected_x=projectx_product>>>30;
 wire signed[63:0] projected_y=projecty_product>>>30;
 wire[64:0] squared_radius={1'b0,squarex_product}+{1'b0,squarey_product};
 function[5:0] leading_bit;input[47:0] value;integer j;
 begin leading_bit=0;for(j=0;j<47;j=j+1) if(value[j]) leading_bit=j;end endfunction
 reg signed[63:0] normalized_x,normalized_y,normalized_z;
 always @* begin
  normalized_x={{16{rx0[47]}},rx0};normalized_y={{16{ry0[47]}},ry0};
  normalized_z={{16{rz0[47]}},rz0};
  if(lead0>=28) begin normalized_x=normalized_x>>>(lead0-28);normalized_y=normalized_y>>>(lead0-28);end
  else begin normalized_x=normalized_x<<<(28-lead0);normalized_y=normalized_y<<<(28-lead0);end
  if(lead0>=30) normalized_z=normalized_z>>>(lead0-30);else normalized_z=normalized_z<<<(30-lead0);
 end
 // Check BEFORE a left shift: otherwise a very small positive Z could shift a
 // large X/Y out of the 64-bit temporary and falsely turn it into a valid ray.
 wire signed[63:0] normalized_limit=64'sd1<<<(lead0+6'd3);
 wire signed[63:0] rx_extended={{16{rx0[47]}},rx0};
 wire signed[63:0] ry_extended={{16{ry0[47]}},ry0};
 wire norm_overflow=rx_extended>=normalized_limit||rx_extended< -normalized_limit||
  ry_extended>=normalized_limit||ry_extended< -normalized_limit;
 integer i;
 // RAM ports have no reset, and their contents are consumed only after setup.
 always @(posedge clk) begin
  if(state==RECIPW&&div_done) reciprocal[seed_index[9:0]]<=quotient[31:0];
  if(state==RADSTORE) begin
   if(radial_index<4096) radial_lo[radial_index[11:0]]<=radial_word;
   if(radial_index!=0) radial_hi[radial_index-1'b1]<=radial_word;
  end
  if(pipe_advance) begin
   seedpipe[2]<=reciprocal[mantissa1[29:20]];
   lower19<=radial_lo[radius2_18[60:49]];upper19<=radial_hi[radius2_18[60:49]];
  end
 end
 always @(posedge clk) if(pipe_advance) begin
  rx0<=rayx;ry0<=rayy;rz0<=rayz;lead0<=leading_bit(rayz);
  xp[1]<=normalized_x[31:0];yp[1]<=normalized_y[31:0];mantissa1<=normalized_z[31:0];
  for(i=2;i<=28;i=i+1) begin xp[i]<=xp[i-1];yp[i]<=yp[i-1];end
  mantissa2<=mantissa1;
  for(i=3;i<=6;i=i+1) seedpipe[i]<=seedpipe[i-1];
  correction6<=32'sd2147483647-$signed(mr_product>>>30)+32'sd1;
  reciprocal10<=nr_product>>>30;
  xp[14]<=projected_x[31:0];yp[14]<=projected_y[31:0];
  radius2_18<=squared_radius[63:0];
  fraction19<=radius2_18[48:29];
  difference20<=$signed(upper19[31:0])-$signed(lower19[31:0]);fraction20<={12'b0,fraction19};
  lowerpipe[20]<=lower19[31:0];
  for(i=21;i<=24;i=i+1) lowerpipe[i]<=lowerpipe[i-1];
  scale24<=lowerpipe[23]+$signed(interpolation_product>>>20);
  xp[28]<=distortx_product>>>30;yp[28]<=distorty_product>>>30;
  us32<=focalx_product>>>16;vs32<=focaly_product>>>16;
  source33u<=us32+($signed({{32{cx[31]}},cx})<<<12);
  source33v<=vs32+($signed({{32{cy[31]}},cy})<<<12);
  identity_pipe[0]<={px[10:0],8'b0,py[10:0],8'b0};
  for(i=1;i<FINAL_STAGE;i=i+1) identity_pipe[i]<=identity_pipe[i-1];
  // mode=0 takes the original pixel-coordinate stream. In particular it
  // bypasses all fisheye/tangential results and their invalid flags.
  if(pidentity[SOURCE_STAGE]) entry34<={1'b1,identity_pipe[SOURCE_STAGE]};
  else if(!pbad[SOURCE_STAGE]&&source33u>=0&&source33u<=MAXX&&source33v>=0&&source33v<=MAXY)
   entry34<={1'b1,rounded_u[38:20],rounded_v[38:20]};
  else entry34<=0;
  entry35<=entry34;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin pv<=0;pl<=0;pbad<=0;pidentity<=0;end
  else if(advance) begin
   pv<={pv[FINAL_STAGE-1:0],issue};pl<={pl[FINAL_STAGE-1:0],issue&&px==IMG_WIDTH-1&&py==IMG_HEIGHT-1};
   pidentity<={pidentity[FINAL_STAGE-1:0],identity};
   pbad<={pbad[FINAL_STAGE-1:0],configuration_bad||ray_bad||rayz<48'sd67108864};
   pbad[1]<=pbad[0]||norm_overflow;
   pbad[14]<=pbad[13]||projected_x>LIMIT28||projected_x< -LIMIT28||projected_y>LIMIT28||projected_y< -LIMIT28;
   pbad[18]<=pbad[17]||squared_radius[64:61]!=0;
   pbad[20]<=pbad[19]||!lower19[32]||!upper19[32];
   if(TANGENTIAL_ENABLED)
    pbad[29+TANGENTIAL_LATENCY]<=pbad[28+TANGENTIAL_LATENCY]||
     (tangential_bad&&!pidentity[28+TANGENTIAL_LATENCY]);
  end
 end
 // Setup multiplication dispatcher. Product width is checked before truncating.
 task multiply;input signed[47:0] a,b;input[5:0] shift_by;input[4:0] destination;
 begin ma<=a;mb<=b;mshift<=shift_by;mwait<=5;return_state<=destination;state<=MULWAIT;end endtask
 wire signed[95:0] shifted_product=mp>>>mshift;
 (* syn_preserve=1 *) reg signed[49:0] composed_origin_r;
 wire signed[49:0] composed_origin=($signed({{18{h[row*3+2][31]}},h[row*3+2]})<<<16)-
  $signed({{2{xc[47]}},xc})-$signed({{2{mresult[47]}},mresult});
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;return_state<=IDLE;mwait<=0;mshift<=0;ma<=0;mb<=0;mresult<=0;
   fx<=0;fy<=0;cx<=0;cy<=0;ox<=0;oy<=0;ofx<=0;ofy<=0;ca<=0;sa<=0;
   invfx<=0;invfy<=0;temporary<=0;ux<=0;vy<=0;xc<=0;yc<=0;row<=0;compose_step<=0;composed_origin_r<=0;
   identity<=0;configuration_bad<=0;issued_all<=0;px<=0;py<=0;
   start_invalid_flags<=0;start_matrix_scale_flags<=0;
   seed_ready<=0;lut_ready<=0;reuse_lut<=0;
   rayx<=0;rayy<=0;rayz<=0;rowx<=0;rowy<=0;rowz<=0;ray_bad<=0;row_bad<=0;
   seed_index<=0;radial_index<=0;radial_step<=0;radius<=0;theta<=0;theta2<=0;horner<=0;theta_d<=0;radial_word<=0;
   done<=0;error<=0;
   for(i=0;i<9;i=i+1) h[i]<=0;
   for(i=0;i<4;i=i+1) begin k[i]<=0;cached_k[i]<=0;end
   for(i=0;i<3;i=i+1) begin dx[i]<=0;dy[i]<=0;origin[i]<=0;end
  end else begin
   done<=0;
   case(state)
    IDLE:if(start) begin
     fx<=cfg_fx_src;fy<=cfg_fy_src;cx<=cfg_cx_src;cy<=cfg_cy_src;
     ox<=cfg_cx_out;oy<=cfg_cy_out;ofx<=cfg_fx_out;ofy<=cfg_fy_out;ca<=cfg_cos;sa<=cfg_sin;
     k[0]<=$signed({{16{cfg_k1[31]}},cfg_k1})<<<12;k[1]<=$signed({{16{cfg_k2[31]}},cfg_k2})<<<12;
     k[2]<=$signed({{16{cfg_k3[31]}},cfg_k3})<<<12;k[3]<=$signed({{16{cfg_k4[31]}},cfg_k4})<<<12;
     for(i=0;i<9;i=i+1) h[i]<=cfg_projective[i*32+:32];
     error<=0;configuration_bad<=0;identity<=cfg_mode==0;px<=0;py<=0;issued_all<=0;
     reuse_lut<=lut_ready&&cfg_k1==cached_k[0]&&cfg_k2==cached_k[1]&&cfg_k3==cached_k[2]&&cfg_k4==cached_k[3];
     ray_bad<=0;row_bad<=0;row<=0;compose_step<=0;seed_index<=0;radial_index<=0;
     start_invalid_flags[0]<=cfg_mode!=1;
     start_invalid_flags[1]<=cfg_fx_src<=0||cfg_fx_src>32'sd268435456;
     start_invalid_flags[2]<=cfg_fy_src<=0||cfg_fy_src>32'sd268435456;
     start_invalid_flags[3]<=cfg_fx_out<32'sd2097152;
     start_invalid_flags[4]<=cfg_fy_out<32'sd2097152;
     start_invalid_flags[5]<=cfg_cos>65536||cfg_cos< -65536;
     start_invalid_flags[6]<=cfg_sin>65536||cfg_sin< -65536;
     start_invalid_flags[7]<=cfg_k1>524288||cfg_k1< -524288;
     start_invalid_flags[8]<=cfg_k2>524288||cfg_k2< -524288;
     start_invalid_flags[9]<=cfg_k3>524288||cfg_k3< -524288;
     start_invalid_flags[10]<=cfg_k4>524288||cfg_k4< -524288;
     for(i=0;i<9;i=i+1)
      start_invalid_flags[11+i]<=$signed(cfg_projective[i*32+:32])>524288||$signed(cfg_projective[i*32+:32])< -524288;
     start_matrix_scale_flags<=matrix_scale_flags;
     if(cfg_mode==0) state<=RUN;else state<=INVX;
    end
    INVX:begin
     // Valid setup starts its divider on the original cycle. Invalid setup
     // starts no arithmetic and reports its all-invalid map one clock later.
     if(start_configuration_bad) begin configuration_bad<=1;error<=1;state<=RUN;end
     else state<=INVXW;
    end
    INVXW:if(div_done) begin invfx<=quotient[47:0];state<=INVY;end
    INVY:state<=INVYW;
    INVYW:if(div_done) begin invfy<=quotient[47:0];state<=COMPOSE;end
    MULWAIT:if(mwait==1) begin
     mresult<=shifted_product[47:0];state<=return_state;
     if(shifted_product[95:48]!={48{shifted_product[47]}}) begin configuration_bad<=1;error<=1;state<=RUN;end
    end else mwait<=mwait-1'b1;
    COMPOSE:begin
     compose_step<=compose_step+1'b1;
     case(compose_step)
      0:multiply(h[row*3],ca,0,COMPOSE);
      1:begin temporary<=mresult;multiply(h[row*3+1],sa,0,COMPOSE);end
      2:begin ux<=(temporary+mresult)>>>16;multiply(h[row*3],sa,0,COMPOSE);end
      3:begin temporary<=mresult;multiply(h[row*3+1],ca,0,COMPOSE);end
      4:begin vy<=(mresult-temporary)>>>16;multiply(ux,invfx,24,COMPOSE);end
      5:begin dx[row]<=mresult;multiply(vy,invfy,24,COMPOSE);end
      6:begin dy[row]<=mresult;multiply(dx[row],ox,16,COMPOSE);end
      7:begin xc<=mresult;multiply(dy[row],oy,16,COMPOSE);end
      8:begin
       // The p branch registers the wide composed-origin result before the
       // overflow/state decision. This splits the 100 MHz mapper control path
       // without changing the p=0 legacy branch or the per-pixel stream.
       if(TANGENTIAL_ENABLED) begin
        composed_origin_r<=composed_origin;state<=COMPOSE_CHECK;
       end else begin
        origin[row]<=composed_origin[47:0];compose_step<=0;
        if(composed_origin[49:48]!={2{composed_origin[47]}}) begin configuration_bad<=1;error<=1;state<=RUN;end
        else if(row==2) begin
         if(reuse_lut) state<=PREPARE_RUN;
         else begin state<=seed_ready?RADSTART:RECIP;seed_index<=0;lut_ready<=0;end
        end else row<=row+1'b1;
       end
      end
     endcase
    end
     COMPOSE_CHECK:begin
      origin[row]<=composed_origin_r[47:0];compose_step<=0;
      if(composed_origin_r[49:48]!={2{composed_origin_r[47]}}) begin configuration_bad<=1;error<=1;state<=RUN;end
      else if(row==2) begin
       if(reuse_lut) state<=PREPARE_RUN;
       else begin state<=seed_ready?RADSTART:RECIP;seed_index<=0;lut_ready<=0;end
      end else begin row<=row+1'b1;state<=COMPOSE;end
     end
    RECIP:state<=RECIPW;
    RECIPW:if(div_done) begin
     if(seed_index==1023) begin radial_index<=0;state<=RADSTART;seed_ready<=1;end
     else begin seed_index<=seed_index+1'b1;state<=RECIP;end
    end
    RADSTART:if(radial_index==0) begin radial_word<=33'h140000000;state<=RADSTORE;end else state<=SQRTW;
    SQRTW:if(sqrt_done) begin radius<=sqrt_root;state<=ATANSTART;end
    ATANSTART:state<=ATANW;
    ATANW:if(atan_done) begin theta<=$signed(atan_theta)>>>2;radial_step<=0;state<=RADMUL;end
    RADMUL:begin
     radial_step<=radial_step+1'b1;
     case(radial_step)
      0:multiply(theta,theta,28,RADMUL);
      1:begin theta2<=mresult;multiply(k[3],mresult,28,RADMUL);end
      2:multiply(mresult+k[2],theta2,28,RADMUL);
      3:multiply(mresult+k[1],theta2,28,RADMUL);
      4:multiply(mresult+k[0],theta2,28,RADMUL);
      5:multiply(mresult+ONE28,theta,28,RADMUL);
      6:begin
       theta_d<=mresult;
       if(mresult<0||mresult>48'sd2147483647) begin radial_word<=0;state<=RADSTORE;end
       else state<=RADDIV;
      end
     endcase
    end
    RADDIV:state<=RADDIVW;
    RADDIVW:if(div_done) begin
     radial_word<=(!div_error&&quotient<64'd536870912)?{1'b1,quotient[29:0],2'b0}:33'd0;state<=RADSTORE;
    end
    RADSTORE:if(radial_index==4096) begin
     state<=PREPARE_RUN;lut_ready<=1;
     for(i=0;i<4;i=i+1)cached_k[i]<=k[i]>>>12;
    end else begin radial_index<=radial_index+1'b1;state<=RADSTART;end
    PREPARE_RUN:begin
     state<=RUN;rayx<=origin[0];rayy<=origin[1];rayz<=origin[2];rowx<=origin[0];rowy<=origin[1];rowz<=origin[2];
    end
    RUN:begin
     if(advance&&issue) begin
      if(px==IMG_WIDTH-1) begin
       px<=0;py<=py+1'b1;rowx<=nextrowx[47:0];rowy<=nextrowy[47:0];rowz<=nextrowz[47:0];
       rayx<=nextrowx[47:0];rayy<=nextrowy[47:0];rayz<=nextrowz[47:0];
       row_bad<=row_bad||(nextrowx[48]!=nextrowx[47])||(nextrowy[48]!=nextrowy[47])||(nextrowz[48]!=nextrowz[47]);
       ray_bad<=row_bad||(nextrowx[48]!=nextrowx[47])||(nextrowy[48]!=nextrowy[47])||(nextrowz[48]!=nextrowz[47]);
       if(py==IMG_HEIGHT-1) issued_all<=1;
      end else begin
       px<=px+1'b1;rayx<=nextx[47:0];rayy<=nexty[47:0];rayz<=nextz[47:0];
       ray_bad<=ray_bad||(nextx[48]!=nextx[47])||(nexty[48]!=nexty[47])||(nextz[48]!=nextz[47]);
      end
     end
     if(advance&&pv[FINAL_STAGE]&&pl[FINAL_STAGE]) begin done<=1;state<=IDLE;end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

// Fixed C97 joint-candidate tangential correction after the fisheye radius
// polynomial. Inputs/outputs are signed Q28. p1=-13/65536, p2=18/65536.
// No configurable or generic p multiplier is inferred: only x*x, y*y and x*y
// use DSPs. All arithmetic, including the three mul32 pipelines, shares ce.
// Eight enabled edges from sampling x/y to output; throughput one pair/edge.
//
// Exact integer numerators (Q56 times the Q16 integer coefficients):
//   dx_num=54*x*x+18*y*y-26*x*y
//   dy_num=-13*x*x-39*y*y+36*x*y
// The delta is floor(numerator/2^44), including negative values; no rounding
// is done inside the constant-product/add tree. Sign extension precedes every
// shift. 72 bits cover every signed 32-bit x/y input (numerators need <=70).
// Overflow is tested on the full wide sum BEFORE narrowing to Q28. Invalid
// results output zero and assert overflow; mapper pbad then emits entry=0.
module complete_tangential_q28(
 input wire clk,ce,
 input wire signed[31:0] x_q28,y_q28,
 output reg signed[31:0] x_out_q28,y_out_q28,
 output reg overflow
);
 wire signed[63:0] xx,yy,xy;
 complete_geometry_mul32 square_x(clk,ce,x_q28,x_q28,xx);
 complete_geometry_mul32 square_y(clk,ce,y_q28,y_q28,yy);
 complete_geometry_mul32 cross_xy(clk,ce,x_q28,y_q28,xy);
 wire signed[71:0] xx_w={{8{xx[63]}},xx};
 wire signed[71:0] yy_w={{8{yy[63]}},yy};
 wire signed[71:0] xy_w={{8{xy[63]}},xy};
 reg signed[31:0] qx[0:6],qy[0:6];
 reg signed[71:0] xx56_4,yy18_4,xy28_4,xx14_4,yy40_4,xy36_4;
 reg signed[63:0] xx4,yy4,xy4;
 wire signed[71:0] xx4_w={{8{xx4[63]}},xx4};
 wire signed[71:0] yy4_w={{8{yy4[63]}},yy4};
 wire signed[71:0] xy4_w={{8{xy4[63]}},xy4};
 reg signed[71:0] xx54_5,yy18_5,xy26_5,xx13_5,yy39_5,xy36_5;
 reg signed[71:0] dx_sum6,dx_cross6,dy_sum6,dy_cross6,dx_num7,dy_num7;
 wire signed[71:0] x_sum8=$signed({{40{qx[6][31]}},qx[6]})+(dx_num7>>>44);
 wire signed[71:0] y_sum8=$signed({{40{qy[6][31]}},qy[6]})+(dy_num7>>>44);
 wire result_overflow=x_sum8[71:32]!={40{x_sum8[31]}}||
  y_sum8[71:32]!={40{y_sum8[31]}};
 integer qi;
 always @(posedge clk) if(ce) begin
  qx[0]<=x_q28;qy[0]<=y_q28;
  for(qi=1;qi<7;qi=qi+1) begin qx[qi]<=qx[qi-1];qy[qi]<=qy[qi-1];end
  // Stage 4: one wide add/subtract for each partial constant product.
  xx56_4<=(xx_w<<<6)-(xx_w<<<3);
  yy18_4<=(yy_w<<<4)+(yy_w<<<1);
  xy28_4<=(xy_w<<<5)-(xy_w<<<2);
  xx14_4<=(xx_w<<<4)-(xx_w<<<1);
  yy40_4<=(yy_w<<<5)+(yy_w<<<3);
  xy36_4<=(xy_w<<<5)+(xy_w<<<2);
  xx4<=xx;yy4<=yy;xy4<=xy;
  // Stage 5: complete coefficients without a chained wide addition.
  xx54_5<=xx56_4-(xx4_w<<<1);yy18_5<=yy18_4;
  xy26_5<=xy28_4-(xy4_w<<<1);
  xx13_5<=xx14_4-xx4_w;yy39_5<=yy40_4-yy4_w;xy36_5<=xy36_4;
  // Stages 6/7: registered two-level reduction, one add/subtract per edge.
  dx_sum6<=xx54_5+yy18_5;dx_cross6<=xy26_5;
  dy_sum6<=xx13_5+yy39_5;dy_cross6<=xy36_5;
  dx_num7<=dx_sum6-dx_cross6;dy_num7<=dy_cross6-dy_sum6;
  // Stage 8: floor delta, add the original q, and reject overflow atomically.
  overflow<=result_overflow;
  x_out_q28<=result_overflow?32'sd0:x_sum8[31:0];
  y_out_q28<=result_overflow?32'sd0:y_sum8[31:0];
 end
endmodule
