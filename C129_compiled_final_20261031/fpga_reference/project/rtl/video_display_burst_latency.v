// DDR burst prefetch into two immutable dual-clock line buffers.
// An HDMI raster never stalls; a late line is black and is released afterwards.
module video_display_burst_latency #(parameter IMG_WIDTH=1920,IMG_HEIGHT=1080,FINAL_OUTPUT_DELAY=1,OBSERVATION_SUPPORT=0,DETAIL_SUPPORT=0)(
 input wire clk,rst_n,pix_clk,pix_rst_n,
 input wire hs,vs,de,input wire[15:0] x,y,
 input wire front_valid,input wire[31:0] front_base,output reg frame_event,
 output wire cmd_valid,cmd_write,output wire[31:0] cmd_addr,output wire[3:0] cmd_len,
 input wire cmd_ready,input wire r_valid,input wire[255:0] r_data,
 input wire r_last,output wire r_ready,input wire done,error,
 output reg out_hs,out_vs,out_de,output reg[15:0] out_rgb565,
 output reg out_observation,out_raw,
 input wire output_detail_enable,
 output reg[31:0] underflow_lines,output reg error_sticky,
 input wire[255:0] diagnostic_front_metadata,input wire[31:0] diagnostic_time_gray,
 input wire[31:0] diagnostic_missing_capture_tags,diagnostic_capture_overrun,
 output wire latency_valid,output wire[511:0] latency_sample
);
 localparam WORDS_PER_LINE=IMG_WIDTH/16;
 localparam WAIT_FRAME=0,WAIT_SELECT=1,SELECT=2,WAIT_FREE=3,ISSUE=4,READ=5;
 reg[2:0] state;
 reg[15:0] next_row,fill_x;reg fill_epoch,fill_bank,discard;
 reg[31:0] row_base;
 reg[1:0] producer,consumer;
 reg[15:0] tag_row[0:1];reg tag_epoch[0:1];
 (* syn_preserve=1, syn_srlstyle="registers" *) reg[1:0] p1,p2,c1,c2;
 (* syn_preserve=1, syn_srlstyle="registers" *) reg front_sync1,front_sync2;
 (* syn_preserve=1, syn_srlstyle="registers" *) reg e1,e2;
 reg epoch,vs_d,line_ok,display_armed,epoch_seen;
 wire bank=y[0];
 wire available=(p2[bank]!=consumer[bank]) && tag_row[bank]==y && tag_epoch[bank]==epoch;
 wire pixel_good=(x==0)?available:line_ok;
 wire[255:0] lane_pixels;reg[3:0] selected_lane;
 wire selected_observation;
 wire selected_raw;
 reg observation_bad,footer_invalid;
 wire observation_area=OBSERVATION_SUPPORT&&selected_observation&&x<320&&y<192;
 genvar k;
 generate for(k=0;k<16;k=k+1) begin: lanes
  reg[15:0] mem[0:2*WORDS_PER_LINE-1];reg[15:0] pixel;
  always @(posedge clk)
   if(state==READ && r_valid && rst_n && !discard && e2==fill_epoch)
    mem[fill_bank*WORDS_PER_LINE+(fill_x>>4)]<=r_data[k*16+:16];
  always @(posedge pix_clk)
   if(de && pixel_good && pix_rst_n) pixel<=mem[bank*WORDS_PER_LINE+(x>>4)];
   else pixel<=0;
  assign lane_pixels[k*16+:16]=pixel;
 end endgenerate
 always @(posedge pix_clk) selected_lane<=pix_rst_n?x[3:0]:4'd0;
 always @* out_rgb565=footer_invalid?16'b0:lane_pixels[selected_lane*16+:16];
 always @(posedge pix_clk or negedge pix_rst_n)
 if(!pix_rst_n) begin
  p1<=0;p2<=0;consumer<=0;epoch<=0;vs_d<=0;line_ok<=0;
  out_hs<=0;out_vs<=0;out_de<=0;underflow_lines<=0;
  out_observation<=0;out_raw<=1;observation_bad<=0;footer_invalid<=0;
  front_sync1<=0;front_sync2<=0;display_armed<=0;
 end else begin
  p1<=producer;p2<=p1;vs_d<=vs;
  front_sync1<=front_valid;front_sync2<=front_sync1;
  out_hs<=hs;out_vs<=vs;out_de<=de;
  out_observation<=de&&observation_area;
  out_raw<=selected_raw;
  footer_invalid<=de&&observation_area&&y>=180&&observation_bad;
  // An unavailable source-window line or late/missing frame metadata makes
  // the in-band code undecodable. Never certify a torn observation region.
  if(vs&&!vs_d)observation_bad<=0;
  if(de&&x==0&&y==0&&!selected_observation)observation_bad<=1;
  if(de&&observation_area&&y<180&&!pixel_good)observation_bad<=1;
  if(vs&&!vs_d) begin epoch<=~epoch;consumer<=p2;line_ok<=0;display_armed<=front_sync2;end
  if(!de) begin
   if(tag_epoch[0]!=epoch) consumer[0]<=p2[0];
   if(tag_epoch[1]!=epoch) consumer[1]<=p2[1];
  end
  if(de&&x==0) begin line_ok<=available;if(!available&&display_armed) underflow_lines<=underflow_lines+1'b1;end
  if(de) begin
   if(p2[0]!=consumer[0]&&tag_epoch[0]==epoch&&tag_row[0]<y) consumer[0]<=p2[0];
   if(p2[1]!=consumer[1]&&tag_epoch[1]==epoch&&tag_row[1]<y) consumer[1]<=p2[1];
  end
  if(de&&x==IMG_WIDTH-1) consumer[bank]<=p2[bank];
 end
 wire[15:0] words_remaining=WORDS_PER_LINE-(fill_x>>4);
 assign cmd_valid=rst_n&&state==ISSUE;
 assign cmd_write=1'b0;
 assign cmd_addr=row_base+(fill_x<<1);
 assign cmd_len=words_remaining>=16?4'd15:words_remaining[3:0]-1'b1;
 assign r_ready=rst_n&&state==READ;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  c1<=0;c2<=0;e1<=0;e2<=0;epoch_seen<=0;producer<=0;
  tag_row[0]<=0;tag_row[1]<=0;tag_epoch[0]<=0;tag_epoch[1]<=0;
  state<=WAIT_FRAME;next_row<=0;fill_x<=0;fill_epoch<=0;
  fill_bank<=0;row_base<=0;discard<=0;frame_event<=0;error_sticky<=0;
 end else begin
  c1<=consumer;c2<=c1;e1<=epoch;e2<=e1;frame_event<=0;
  if(done&&error) error_sticky<=1;
  if(e2!=epoch_seen) begin
   epoch_seen<=e2;frame_event<=1;discard<=1;
   if(state==READ && !(r_valid&&r_last)) state<=READ;
   // A presented descriptor cannot be withdrawn at a frame boundary. The
   // arbiter may already have locked it while the bridge is backpressured.
   // Keep address/length stable through acceptance, then drain with discard.
   else if(state==ISSUE) state<=cmd_ready?READ:ISSUE;
   else state<=WAIT_SELECT;
  end else case(state)
   WAIT_FRAME: ;
   WAIT_SELECT: state<=SELECT;
   SELECT: begin
    row_base<=front_base;next_row<=0;fill_x<=0;fill_epoch<=e2;discard<=0;
    state<=front_valid?WAIT_FREE:WAIT_FRAME;
   end
   WAIT_FREE: if(producer[next_row[0]]==c2[next_row[0]]) begin
    fill_bank<=next_row[0];fill_x<=0;state<=ISSUE;
   end
   ISSUE: if(cmd_ready) state<=READ;
   READ: if(r_valid) begin
    if(discard) begin if(r_last) state<=WAIT_SELECT;end
    else begin
     fill_x<=fill_x+16;
     if(r_last) begin
      if(fill_x+16>=IMG_WIDTH) begin
       tag_row[fill_bank]<=next_row;tag_epoch[fill_bank]<=fill_epoch;
       producer[fill_bank]<=~producer[fill_bank];next_row<=next_row+1'b1;
       row_base<=row_base+IMG_WIDTH*2;
       state<=(next_row==IMG_HEIGHT-1)?WAIT_FRAME:WAIT_FREE;
      end else state<=ISSUE;
     end
    end
   end
   default: state<=WAIT_FRAME;
  endcase
 end

 latency_display_observer #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.FINAL_OUTPUT_DELAY(FINAL_OUTPUT_DELAY),.DETAIL_SUPPORT(DETAIL_SUPPORT)) diagnostics(
  .clk(clk),.rst_n(rst_n),.select_frame(state==SELECT&&front_valid&&e2==epoch_seen),.new_core_epoch(e2!=epoch_seen),
  .front_metadata(diagnostic_front_metadata),.time_gray(diagnostic_time_gray),
  .pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.new_pix_epoch(vs&&!vs_d),
  .de(de),.pixel_good(pixel_good),.x(x),.y(y),.underflow_lines(underflow_lines),
  .missing_capture_tags(diagnostic_missing_capture_tags),.capture_metadata_overrun(diagnostic_capture_overrun),
  .latency_valid(latency_valid),.latency_sample(latency_sample),.selected_observation(selected_observation),
  .selected_raw(selected_raw),.output_detail_enable(output_detail_enable));

endmodule
