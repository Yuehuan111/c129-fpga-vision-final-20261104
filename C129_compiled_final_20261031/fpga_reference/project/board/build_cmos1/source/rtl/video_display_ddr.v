// Two complete line buffers decouple DDR service from an unstoppable HDMI raster.
// Core-side DDR reads have display priority. Underflow outputs black, never pauses HDMI.
// width must be divisible by 16 (one 256-bit beat = sixteen RGB565 pixels).
module video_display_ddr #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input wire clk,rst_n,pix_clk,pix_rst_n,
 input wire hs,vs,de,input wire[15:0] x,y,
 input wire front_valid,input wire[31:0] front_base,
 output reg frame_event,
 output wire service_active,
 output wire req_valid,output wire[31:0] req_addr,input wire req_ready,
 input wire rsp_valid,input wire[255:0] rsp_data,output wire rsp_ready,
 output reg out_hs,out_vs,out_de,output reg[15:0] out_rgb565,
 output reg[31:0] underflow_lines
);
 localparam WORDS_PER_LINE=IMG_WIDTH/16;
 wire[255:0] lane_pixels;
 reg[3:0] selected_lane;
 reg[1:0] producer,consumer;
 reg[15:0] tag_row[0:1];reg tag_epoch[0:1];
 (* ASYNC_REG="TRUE" *) reg[1:0] p1,p2,c1,c2;
 reg epoch,vs_d,line_ok;
 (* ASYNC_REG="TRUE" *) reg front_sync1,front_sync2;
 reg display_armed;
 (* ASYNC_REG="TRUE" *) reg e1,e2;
 reg epoch_seen;
 wire bank=y[0];
 wire available=(p2[bank]!=consumer[bank]) && tag_row[bank]==y && tag_epoch[bank]==epoch;
 wire pixel_good=(x==0)?available:line_ok;
 localparam WAIT_FRAME=0,WAIT_SELECT=1,SELECT=2,WAIT_FREE=3,ISSUE=4,READ=5;
 reg[2:0] state;
 reg[15:0] next_row,fill_x;
 reg fill_epoch,fill_bank;reg[31:0] base;
 reg discard;
 // Payload/tag crossing is bundled with producer toggle; payload stays immutable
 // until the consumer has released that entire line and the release crosses back.
 // Sixteen independently inferred dual-clock RAM lanes accept one complete
 // DDR beat at once. Reading all lanes at x/16 preserves the original
 // one-pixel-clock latency; selected_lane follows the same sampled x.
 genvar k;
 generate for(k=0;k<16;k=k+1) begin: lanes
  reg[15:0] mem[0:2*WORDS_PER_LINE-1];
  reg[15:0] pixel;
  always @(posedge clk)
   if(state==READ && rsp_valid && rst_n && !discard && e2==fill_epoch)
    mem[fill_bank*WORDS_PER_LINE+(fill_x>>4)]<=rsp_data[k*16+:16];
  always @(posedge pix_clk)
   if(de && pixel_good && pix_rst_n) pixel<=mem[bank*WORDS_PER_LINE+(x>>4)];
   else pixel<=0;
  assign lane_pixels[k*16+:16]=pixel;
 end endgenerate
 always @(posedge pix_clk) selected_lane<=pix_rst_n?x[3:0]:4'd0;
 always @* out_rgb565=lane_pixels[selected_lane*16+:16];
 always @(posedge pix_clk or negedge pix_rst_n)
 if(!pix_rst_n) begin
  p1<=0;p2<=0;consumer<=0;epoch<=0;vs_d<=0;line_ok<=0;
  out_hs<=0;out_vs<=0;out_de<=0;underflow_lines<=0;
  front_sync1<=0;front_sync2<=0;display_armed<=0;
 end else begin
  p1<=producer;p2<=p1;vs_d<=vs;
  front_sync1<=front_valid;front_sync2<=front_sync1;
  out_hs<=hs;out_vs<=vs;out_de<=de;
  if(vs&&!vs_d) begin epoch<=~epoch;consumer<=p2;line_ok<=0;display_armed<=front_sync2;end
  if(!de) begin
   if(tag_epoch[0]!=epoch) consumer[0]<=p2[0];
   if(tag_epoch[1]!=epoch) consumer[1]<=p2[1];
  end
  if(de&&x==0) begin
   line_ok<=available;
   if(!available && display_armed) underflow_lines<=underflow_lines+1'b1;
  end
  // Release a late line after its raster deadline instead of holding a stale
  // row in a bank until the next frame. The payload remains immutable until
  // this release crosses back to the producer clock domain.
  if(de) begin
   if(p2[0]!=consumer[0] && tag_epoch[0]==epoch && tag_row[0]<y)
    consumer[0]<=p2[0];
   if(p2[1]!=consumer[1] && tag_epoch[1]==epoch && tag_row[1]<y)
    consumer[1]<=p2[1];
  end
  if(de&&x==IMG_WIDTH-1) consumer[bank]<=p2[bank];
 end
 assign service_active=rst_n && (state==ISSUE || state==READ);
 assign req_valid=rst_n && state==ISSUE;
 assign req_addr=base+((next_row*IMG_WIDTH+fill_x)<<1);
 assign rsp_ready=rst_n && state==READ;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  c1<=0;c2<=0;e1<=0;e2<=0;epoch_seen<=0;producer<=0;
  tag_row[0]<=0;tag_row[1]<=0;tag_epoch[0]<=0;tag_epoch[1]<=0;
  state<=WAIT_FRAME;next_row<=0;fill_x<=0;fill_epoch<=0;
  fill_bank<=0;base<=0;discard<=0;frame_event<=0;
 end else begin
  c1<=consumer;c2<=c1;e1<=epoch;e2<=e1;frame_event<=0;
  // Finish a previously accepted DDR transaction before abandoning an old line.
  if(e2!=epoch_seen) begin
   epoch_seen<=e2;frame_event<=1;discard<=1;
   if(state==READ && !rsp_valid) state<=READ;
   else if(state==ISSUE&&req_ready) state<=READ;
   else state<=WAIT_SELECT;
  end else case(state)
   WAIT_FRAME: ;
   WAIT_SELECT: state<=SELECT;
   SELECT: begin
    base<=front_base;next_row<=0;fill_x<=0;fill_epoch<=e2;discard<=0;
    state<=front_valid?WAIT_FREE:WAIT_FRAME;
   end
   WAIT_FREE: if(producer[next_row[0]]==c2[next_row[0]]) begin
    fill_bank<=next_row[0];fill_x<=0;state<=ISSUE;
   end
   ISSUE: if(req_ready) state<=READ;
   READ: if(rsp_valid) begin
    if(discard) state<=WAIT_SELECT;
    else begin
     if(fill_x+16>=IMG_WIDTH) begin
      tag_row[fill_bank]<=next_row;tag_epoch[fill_bank]<=fill_epoch;
      producer[fill_bank]<=~producer[fill_bank];next_row<=next_row+1'b1;
      state<=(next_row==IMG_HEIGHT-1)?WAIT_FRAME:WAIT_FREE;
     end else begin fill_x<=fill_x+16;state<=ISSUE;end
    end
   end
   default: state<=WAIT_FRAME;
  endcase
 end
endmodule
