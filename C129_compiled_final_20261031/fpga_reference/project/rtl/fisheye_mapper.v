// OpenCV fisheye: output pixel -> input pixel. Synthesizable Verilog for PDS.
// Camera parameters/cos/sin are signed Q16.16. Internal geometry is Q28,
// focal reciprocals Q40, CORDIC angles Q30. Output is {valid,xQ11.8,yQ11.8}.
// mode=0 is EXPLICIT identity; mode=1 is fisheye, including zero coefficients.
// One shared registered 48x48 multiplier. No external coordinate ROM is used.
module fisheye_mapper #(
 parameter IMG_WIDTH=1920,IMG_HEIGHT=1080
)(
 input wire clk,rst_n,start,
 input wire signed[31:0] cfg_fx_src,cfg_fy_src,cfg_cx_src,cfg_cy_src,
 input wire signed[31:0] cfg_cx_out,cfg_cy_out,cfg_fx_out,cfg_fy_out,
 input wire signed[31:0] cfg_k1,cfg_k2,cfg_k3,cfg_k4,cfg_cos,cfg_sin,
 input wire[31:0] cfg_mode,
 input wire map_ready,output wire busy,
 output reg done,map_valid,output reg[38:0] map_entry,output reg map_last,error
);
 localparam[5:0] IDLE=0,SQRT_START=30,SQRT_WAIT=31,ATAN_START=32,ATAN_WAIT=33,
  SCALE_START=34,SCALE_WAIT=35,INVX_START=36,INVX_WAIT=37,INVY_START=38,INVY_WAIT=39,
  PACK=60,SEND=61,IDENTITY=62;
 localparam[1:0] OP_NONE=0,OP_MUL=1,OP_ADD=2,OP_SUB=3;
 localparam signed[47:0] ONE=48'sd268435456,NORM_LIMIT=48'sd1073741824;
 localparam signed[47:0] MAX_X=(IMG_WIDTH-1)*48'sd268435456;
 localparam signed[47:0] MAX_Y=(IMG_HEIGHT-1)*48'sd268435456;
 reg[5:0] step;
 reg[2:0] phase;
 reg[15:0] pixel_x,pixel_y;
 reg configuration_bad,pixel_invalid,identity_mode;
 reg signed[31:0] fx_src,fy_src,cx_src,cy_src,cx_out,cy_out,fx_out,fy_out,cos_a,sin_a;
 reg signed[47:0] k1,k2,k3,k4,inv_fx,inv_fy;
 reg signed[47:0] du,dv,nx,ny,rcx,rsy,rsx,rcy;
 reg[63:0] square_x,square_y;
 reg[31:0] radius;
 reg signed[47:0] theta,theta2,horner,theta_distorted,scale,xd,yd,us,vs,source_u,source_v;
 reg signed[47:0] operand_a,operand_b,selected_a,selected_b;
 reg[1:0] operation,selected_operation;
 reg[5:0] shift,selected_shift;
 wire signed[95:0] multiply_full;
 reg signed[95:0] arithmetic_result;
 fisheye_multiplier48 shared_multiplier(.clk(clk),.a(selected_a),.b(selected_b),.product(multiply_full));
 reg signed[48:0] add_full;
 wire arithmetic_overflow=arithmetic_result[95:48]!={48{arithmetic_result[47]}};
 wire signed[48:0] extended_a={selected_a[47],selected_a};
 wire signed[48:0] extended_b={selected_b[47],selected_b};
 wire[64:0] radius_square={1'b0,square_x}+{1'b0,square_y};
 wire sqrt_done,atan_done,divide_done,divide_error;
 wire[31:0] sqrt_root;
 wire signed[31:0] atan_theta;
 wire[63:0] divide_quotient;
 wire division_start=step==INVX_START||step==INVY_START||step==SCALE_START;
 wire[31:0] divisor=step==INVX_START?fx_out:step==INVY_START?fy_out:radius;
 wire[63:0] dividend=(step==INVX_START||step==INVY_START)?
  64'h0100000000000000:({32'b0,theta_distorted[31:0]}<<28);
 fisheye_unsigned_divider divider(.clk(clk),.rst_n(rst_n),.start(division_start),
  .numerator(dividend),.denominator(divisor),.busy(),.done(divide_done),.error(divide_error),.quotient(divide_quotient));
 fisheye_unsigned_sqrt square_root(.clk(clk),.rst_n(rst_n),.start(step==SQRT_START),
  .radicand(radius_square[63:0]),.busy(),.done(sqrt_done),.root(sqrt_root));
 fisheye_atan arctangent(.clk(clk),.rst_n(rst_n),.start(step==ATAN_START),
  .radius_q28(radius),.busy(),.done(atan_done),.theta_q30(atan_theta));
 assign busy=rst_n&&step!=IDLE;
 always @* begin
  operation=OP_NONE;operand_a=0;operand_b=0;shift=28;
  case(step)
   1:begin operation=OP_SUB;operand_a=$signed({pixel_x,16'b0});operand_b=cx_out;end
   2:begin operation=OP_SUB;operand_a=$signed({pixel_y,16'b0});operand_b=cy_out;end
   3:begin operation=OP_MUL;operand_a=du;operand_b=inv_fx;end
   4:begin operation=OP_MUL;operand_a=dv;operand_b=inv_fy;end
   5:begin operation=OP_MUL;operand_a=nx;operand_b=cos_a;shift=16;end
   6:begin operation=OP_MUL;operand_a=ny;operand_b=sin_a;shift=16;end
   7:begin operation=OP_MUL;operand_a=nx;operand_b=sin_a;shift=16;end
   8:begin operation=OP_MUL;operand_a=ny;operand_b=cos_a;shift=16;end
   9:begin operation=OP_SUB;operand_a=rcx;operand_b=rsy;end
   10:begin operation=OP_ADD;operand_a=rsx;operand_b=rcy;end
   11:begin operation=OP_MUL;operand_a=nx;operand_b=nx;shift=0;end
   12:begin operation=OP_MUL;operand_a=ny;operand_b=ny;shift=0;end
   14:begin operation=OP_MUL;operand_a=theta;operand_b=theta;end
   15:begin operation=OP_MUL;operand_a=k4;operand_b=theta2;end
   16:begin operation=OP_ADD;operand_a=horner;operand_b=k3;end
   17,19,21:begin operation=OP_MUL;operand_a=horner;operand_b=theta2;end
   18:begin operation=OP_ADD;operand_a=horner;operand_b=k2;end
   20:begin operation=OP_ADD;operand_a=horner;operand_b=k1;end
   22:begin operation=OP_ADD;operand_a=horner;operand_b=ONE;end
   23:begin operation=OP_MUL;operand_a=theta;operand_b=horner;end
   24:begin operation=OP_MUL;operand_a=nx;operand_b=scale;end
   25:begin operation=OP_MUL;operand_a=ny;operand_b=scale;end
   26:begin operation=OP_MUL;operand_a=fx_src;operand_b=xd;shift=16;end
   27:begin operation=OP_MUL;operand_a=fy_src;operand_b=yd;shift=16;end
   28:begin operation=OP_ADD;operand_a=us;operand_b=$signed(cx_src);operand_b=operand_b<<<12;end
   29:begin operation=OP_ADD;operand_a=vs;operand_b=$signed(cy_src);operand_b=operand_b<<<12;end
   default:;
  endcase
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  step<=IDLE;phase<=0;pixel_x<=0;pixel_y<=0;configuration_bad<=0;pixel_invalid<=0;identity_mode<=0;
  done<=0;map_valid<=0;map_entry<=0;map_last<=0;error<=0;
  fx_src<=0;fy_src<=0;cx_src<=0;cy_src<=0;cx_out<=0;cy_out<=0;fx_out<=0;fy_out<=0;cos_a<=0;sin_a<=0;
  k1<=0;k2<=0;k3<=0;k4<=0;inv_fx<=0;inv_fy<=0;du<=0;dv<=0;nx<=0;ny<=0;rcx<=0;rsy<=0;rsx<=0;rcy<=0;
  square_x<=0;square_y<=0;radius<=0;theta<=0;theta2<=0;horner<=0;theta_distorted<=0;scale<=0;
  xd<=0;yd<=0;us<=0;vs<=0;source_u<=0;source_v<=0;
  selected_a<=0;selected_b<=0;selected_operation<=0;selected_shift<=0;add_full<=0;arithmetic_result<=0;
 end else begin
  done<=0;
  if(step==IDLE) begin
   if(start) begin
    fx_src<=cfg_fx_src;fy_src<=cfg_fy_src;cx_src<=cfg_cx_src;cy_src<=cfg_cy_src;
    cx_out<=cfg_cx_out;cy_out<=cfg_cy_out;fx_out<=cfg_fx_out;fy_out<=cfg_fy_out;
    k1<=$signed({{16{cfg_k1[31]}},cfg_k1})<<<12;k2<=$signed({{16{cfg_k2[31]}},cfg_k2})<<<12;
    k3<=$signed({{16{cfg_k3[31]}},cfg_k3})<<<12;k4<=$signed({{16{cfg_k4[31]}},cfg_k4})<<<12;
    cos_a<=cfg_cos;sin_a<=cfg_sin;
    pixel_x<=0;pixel_y<=0;phase<=0;error<=0;map_valid<=0;map_last<=0;configuration_bad<=0;pixel_invalid<=0;
    identity_mode<=cfg_mode==0;
    if(cfg_mode==0) step<=IDENTITY;
    else if(cfg_mode!=1||cfg_fx_src<=0||cfg_fy_src<=0||cfg_fx_out<32'sd2097152||cfg_fy_out<32'sd2097152) begin
     configuration_bad<=1;pixel_invalid<=1;error<=1;step<=PACK;
    end else step<=INVX_START;
   end
  end else if(step==INVX_START) step<=INVX_WAIT;
  else if(step==INVX_WAIT&&divide_done) begin
   if(divide_error||divide_quotient[63:47]!=0) begin configuration_bad<=1;pixel_invalid<=1;error<=1;step<=PACK;end
   else begin inv_fx<=divide_quotient[47:0];step<=INVY_START;end
  end else if(step==INVY_START) step<=INVY_WAIT;
  else if(step==INVY_WAIT&&divide_done) begin
   if(divide_error||divide_quotient[63:47]!=0) begin configuration_bad<=1;pixel_invalid<=1;error<=1;step<=PACK;end
   else begin inv_fy<=divide_quotient[47:0];step<=1;end
  end else if(step==5&&cos_a==32'sd65536&&sin_a==0) begin
   // Exact identity rotation has no truncation effects; retain nx/ny.
   step<=11;
  end else if(step==13) begin
   if(nx>NORM_LIMIT||nx< -NORM_LIMIT||ny>NORM_LIMIT||ny< -NORM_LIMIT||radius_square[64]) begin
    pixel_invalid<=1;error<=1;step<=PACK;
   end else step<=SQRT_START;
  end else if(step==SQRT_START) step<=SQRT_WAIT;
  else if(step==SQRT_WAIT&&sqrt_done) begin
   radius<=sqrt_root;
   if(sqrt_root==0) begin scale<=ONE;step<=24;end
   else step<=ATAN_START;
  end else if(step==ATAN_START) step<=ATAN_WAIT;
  else if(step==ATAN_WAIT&&atan_done) begin theta<=$signed(atan_theta)>>>2;step<=14;end
  else if(step==SCALE_START) begin
   if(theta_distorted<0||theta_distorted>48'sd2147483647) begin pixel_invalid<=1;error<=1;step<=PACK;end
   else step<=SCALE_WAIT;
  end else if(step==SCALE_WAIT&&divide_done) begin
   if(divide_error||divide_quotient[63:47]!=0) begin pixel_invalid<=1;error<=1;step<=PACK;end
   else begin scale<=divide_quotient[47:0];step<=24;end
  end else if(step==IDENTITY) begin
   map_entry<={1'b1,pixel_x[10:0],8'b0,pixel_y[10:0],8'b0};
   map_valid<=1;map_last<=pixel_x==IMG_WIDTH-1&&pixel_y==IMG_HEIGHT-1;step<=SEND;
  end else if(step==PACK) begin
   if(!pixel_invalid&&source_u>=0&&source_v>=0&&source_u<=MAX_X&&source_v<=MAX_Y)
    map_entry<={1'b1,source_u[38:20],source_v[38:20]};
   else map_entry<=0;
   map_valid<=1;map_last<=pixel_x==IMG_WIDTH-1&&pixel_y==IMG_HEIGHT-1;step<=SEND;
  end else if(step==SEND) begin
   if(map_valid&&map_ready) begin
    map_valid<=0;map_last<=0;phase<=0;
    if(map_last) begin done<=1;step<=IDLE;end
    else begin
     if(pixel_x==IMG_WIDTH-1) begin pixel_x<=0;pixel_y<=pixel_y+1'b1;end else pixel_x<=pixel_x+1'b1;
     pixel_invalid<=configuration_bad;
     step<=configuration_bad?PACK:identity_mode?IDENTITY:6'd1;
    end
   end
  end else if(operation!=OP_NONE) begin
   case(phase)
    0:begin selected_a<=operand_a;selected_b<=operand_b;selected_operation<=operation;selected_shift<=shift;phase<=1;end
    1:begin add_full<=selected_operation==OP_SUB?extended_a-extended_b:extended_a+extended_b;phase<=selected_operation==OP_MUL?3'd2:3'd7;end
    2,3,4:phase<=phase+1'b1;
    7:begin arithmetic_result<={{47{add_full[48]}},add_full};phase<=6;end
    5:begin
     if(selected_operation==OP_MUL) begin
      case(selected_shift) 0:arithmetic_result<=multiply_full;16:arithmetic_result<=multiply_full>>>16;default:arithmetic_result<=multiply_full>>>28;endcase
     end else arithmetic_result<={{47{add_full[48]}},add_full};
     phase<=6;
    end
    6:begin
     phase<=0;
     if((step==11||step==12)?arithmetic_result[95:64]!=0:arithmetic_overflow) begin pixel_invalid<=1;error<=1;step<=PACK;end
     else begin
      case(step)
       1:du<=arithmetic_result[47:0];2:dv<=arithmetic_result[47:0];
       3:nx<=arithmetic_result[47:0];4:ny<=arithmetic_result[47:0];
       5:rcx<=arithmetic_result[47:0];6:rsy<=arithmetic_result[47:0];7:rsx<=arithmetic_result[47:0];8:rcy<=arithmetic_result[47:0];
       9:nx<=arithmetic_result[47:0];10:ny<=arithmetic_result[47:0];
       11:square_x<=arithmetic_result[63:0];12:square_y<=arithmetic_result[63:0];
       14:theta2<=arithmetic_result[47:0];
       15,16,17,18,19,20,21,22:horner<=arithmetic_result[47:0];
       23:theta_distorted<=arithmetic_result[47:0];24:xd<=arithmetic_result[47:0];25:yd<=arithmetic_result[47:0];
       26:us<=arithmetic_result[47:0];27:vs<=arithmetic_result[47:0];28:source_u<=arithmetic_result[47:0];29:source_v<=arithmetic_result[47:0];
       default:;
      endcase
      step<=step==23?SCALE_START:step==29?PACK:step+1'b1;
     end
    end
   endcase
  end
 end
endmodule
