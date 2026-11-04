// Eight-stage, one-pixel-per-cycle bilinear RGB565 pipeline.
// Pixel order: {p11,p01,p10,p00}. fx/fy are unsigned Q0.8 fractions.
// All arithmetic is exactly equivalent to the original four-weight sum:
// horizontal sums retain all eight fractional bits; rounding happens only
// after vertical interpolation. Gain/offset and metadata travel with each pixel.
// Global clock enable provides lossless ready/valid backpressure. No input
// or output may change while valid && !ready. Invalid map entries output black.
module bilinear_rgb565_pipeline(
 input wire clk,rst_n,
 input wire s_valid,output wire s_ready,
 input wire[63:0] s_pixels,input wire[7:0] s_fx,s_fy,
 input wire s_inside,input wire[15:0] s_x,s_y,input wire s_last,
 input wire[31:0] s_gain_q8,s_offset,
 output wire m_valid,input wire m_ready,
 output reg[15:0] m_rgb565,output wire[15:0] m_x,m_y,output wire m_last
);
 reg[7:0] valid,inside_pipe;
 reg[15:0] xs[0:7],ys[0:7];reg[7:0] lasts;
 reg signed[31:0] gains[0:7],offsets[0:7];
 reg[8:0] fx0,fy0,fy1,fy2;
 reg[7:0] bases0[0:5];reg signed[8:0] diffs0[0:5];
 reg[15:0] bases1[0:5];reg signed[17:0] products1[0:5];
 reg[15:0] horizontal2[0:5];
 reg[23:0] bases3[0:2];reg signed[25:0] products3[0:2];
 reg signed[25:0] sums4[0:2];
 reg signed[23:0] gain_hi5[0:2];
 reg[15:0] gain_lo5[0:2];
 reg signed[63:0] adjust6[0:2];
 wire advance=!valid[7]||m_ready;
 assign s_ready=rst_n&&advance;
 assign m_valid=rst_n&&valid[7];
 assign m_x=xs[7];assign m_y=ys[7];assign m_last=lasts[7];
 function[7:0] component;
  input[15:0] pixel;input integer channel;
  begin case(channel)
   0:component={pixel[15:11],pixel[15:13]};
   1:component={pixel[10:5],pixel[10:9]};
   default:component={pixel[4:0],pixel[4:2]};
  endcase end
 endfunction
 function[7:0] saturate;
  input signed[63:0] value;
  begin if(value<0)saturate=0;else if(value>255)saturate=255;else saturate=value[7:0];end
 endfunction
 wire[7:0] red6=saturate(adjust6[0]),green6=saturate(adjust6[1]),blue6=saturate(adjust6[2]);
 wire signed[16:0] vertical_difference0=$signed({1'b0,horizontal2[1]})-$signed({1'b0,horizontal2[0]});
 wire signed[16:0] vertical_difference1=$signed({1'b0,horizontal2[3]})-$signed({1'b0,horizontal2[2]});
 wire signed[16:0] vertical_difference2=$signed({1'b0,horizontal2[5]})-$signed({1'b0,horizontal2[4]});
 // Exact signed 32-bit gain split. Each multiplier is <=17x9 bits and
 // fits one APM; no vendor-specific DSP attribute or cascade is required.
 wire signed[23:0] gain_hi_product0,gain_hi_product1,gain_hi_product2;
 wire[23:0] gain_lo_product0,gain_lo_product1,gain_lo_product2;
 wire[23:0] gain_lo_biased0=gain_lo_product0+24'd128;
 wire[23:0] gain_lo_biased1=gain_lo_product1+24'd128;
 wire[23:0] gain_lo_biased2=gain_lo_product2+24'd128;
 assign gain_hi_product0=$signed({1'b0,sums4[0][23:16]})*$signed(gains[4][31:16]);
 assign gain_hi_product1=$signed({1'b0,sums4[1][23:16]})*$signed(gains[4][31:16]);
 assign gain_hi_product2=$signed({1'b0,sums4[2][23:16]})*$signed(gains[4][31:16]);
 assign gain_lo_product0=$signed({1'b0,sums4[0][23:16]})*$signed({1'b0,gains[4][15:0]});
 assign gain_lo_product1=$signed({1'b0,sums4[1][23:16]})*$signed({1'b0,gains[4][15:0]});
 assign gain_lo_product2=$signed({1'b0,sums4[2][23:16]})*$signed({1'b0,gains[4][15:0]});
 integer i,c;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   valid<=0;inside_pipe<=0;lasts<=0;m_rgb565<=0;fx0<=0;fy0<=0;fy1<=0;fy2<=0;
   for(i=0;i<8;i=i+1)begin xs[i]<=0;ys[i]<=0;gains[i]<=0;offsets[i]<=0;end
   for(i=0;i<6;i=i+1)begin bases0[i]<=0;diffs0[i]<=0;bases1[i]<=0;products1[i]<=0;horizontal2[i]<=0;end
   for(i=0;i<3;i=i+1)begin bases3[i]<=0;products3[i]<=0;sums4[i]<=0;gain_hi5[i]<=0;gain_lo5[i]<=0;adjust6[i]<=0;end
  end else if(advance)begin
   valid<={valid[6:0],s_valid};inside_pipe<={inside_pipe[6:0],s_inside};lasts<={lasts[6:0],s_last};
   xs[0]<=s_x;ys[0]<=s_y;gains[0]<=s_gain_q8;offsets[0]<=s_offset;
   for(i=1;i<8;i=i+1)begin xs[i]<=xs[i-1];ys[i]<=ys[i-1];gains[i]<=gains[i-1];offsets[i]<=offsets[i-1];end
   fx0<={1'b0,s_fx};fy0<={1'b0,s_fy};fy1<=fy0;fy2<=fy1;
   // Stage 0: signed differences and unsigned endpoints, three colors/two rows.
   for(c=0;c<3;c=c+1)begin
    bases0[c*2]<=component(s_pixels[15:0],c);
    bases0[c*2+1]<=component(s_pixels[47:32],c);
    diffs0[c*2]<=$signed({1'b0,component(s_pixels[31:16],c)})-$signed({1'b0,component(s_pixels[15:0],c)});
    diffs0[c*2+1]<=$signed({1'b0,component(s_pixels[63:48],c)})-$signed({1'b0,component(s_pixels[47:32],c)});
   end
   // Stage 1: six signed 9x9 products. Stage 2 keeps the complete Q8 sums.
   for(i=0;i<6;i=i+1)begin
    products1[i]<=diffs0[i]*$signed(fx0);bases1[i]<={bases0[i],8'b0};
    horizontal2[i]<=$signed({2'b0,bases1[i]})+products1[i];
   end
   // Stage 3: three signed 17x9 products. Stage 4 performs the sole rounding.
   products3[0]<=vertical_difference0*$signed(fy2);
   products3[1]<=vertical_difference1*$signed(fy2);
   products3[2]<=vertical_difference2*$signed(fy2);
   for(i=0;i<3;i=i+1)begin
    bases3[i]<={horizontal2[i*2],8'b0};
    sums4[i]<=$signed({2'b0,bases3[i]})+products3[i]+26'sd32768;
    adjust6[i]<=($signed({{40{gain_hi5[i][23]}},gain_hi5[i]})<<<8)+
                $signed({48'd0,gain_lo5[i]})+$signed(offsets[5]);
   end
   // Stages 5/6 apply fixed gain/offset; stage 7 saturates and repacks.
   gain_hi5[0]<=gain_hi_product0;gain_hi5[1]<=gain_hi_product1;gain_hi5[2]<=gain_hi_product2;
   gain_lo5[0]<=gain_lo_biased0[23:8];gain_lo5[1]<=gain_lo_biased1[23:8];gain_lo5[2]<=gain_lo_biased2[23:8];
   m_rgb565<=inside_pipe[6]?{red6[7:3],green6[7:2],blue6[7:3]}:16'b0;
  end
 end
endmodule

