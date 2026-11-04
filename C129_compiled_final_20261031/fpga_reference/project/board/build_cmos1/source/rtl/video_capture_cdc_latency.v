// A request toggle selects exactly one complete sensor frame. The sensor never stalls.
// Packet bits17:16: 00=pixel, 01=validated frame end, 10=bad frame end.
module video_capture_cdc_latency #(parameter FIFO_ADDR_BITS=11)(
 input wire cam_clk,cam_rst_n,pix_valid,input wire[15:0] rgb565,
 input wire sof,frame_valid_done,frame_drop,input wire request_toggle,
 input wire clk,rst_n,output wire valid,output wire[17:0] data,input wire ready,
 output reg[31:0] overflow_frames,output reg[31:0] skipped_frames,
 input wire[31:0] diagnostic_time_gray,
 output wire diagnostic_tag_valid,diagnostic_tag_epoch,
 output wire[31:0] diagnostic_frame_id,diagnostic_first_us,diagnostic_overruns
);
 (* syn_preserve=1, syn_srlstyle="registers" *) reg req1,req2;
 reg seen,active,poison,end_pending,end_bad;
 wire take_new=sof && (req2!=seen) && !active && !end_pending;
 wire take_pixel=pix_valid && (active || take_new);
 wire fifo_ready;
 wire send=end_pending || take_pixel;
 wire[17:0] word=end_pending ? {end_bad?2'b10:2'b01,16'b0} : {2'b00,rgb565};
 always @(posedge cam_clk or negedge cam_rst_n)
 if(!cam_rst_n) begin
  req1<=0;req2<=0;seen<=0;active<=0;poison<=0;
  end_pending<=0;end_bad<=0;overflow_frames<=0;skipped_frames<=0;
 end else begin
  req1<=request_toggle;req2<=req1;
  if(take_new) begin seen<=req2;active<=1;poison<=0;end
  if(sof&&!take_new) skipped_frames<=skipped_frames+1'b1;
  if(take_pixel && !fifo_ready) poison<=1;
  if(active && (frame_valid_done || frame_drop)) begin
   active<=0;end_pending<=1;end_bad<=poison || frame_drop;
   if(poison) overflow_frames<=overflow_frames+1'b1;
  end
  if(end_pending && fifo_ready) end_pending<=0;
 end
 async_pixel_fifo #(.WIDTH(18),.ADDR_BITS(FIFO_ADDR_BITS)) fifo(
  .wclk(cam_clk),.wrst_n(cam_rst_n),.wvalid(send),.wdata(word),.wready(fifo_ready),
  .rclk(clk),.rrst_n(rst_n),.rvalid(valid),.rdata(data),.rready(ready));

 // Selected sensor frame first pixel, not request issue or FIFO drain time.
 wire[31:0] camera_time;
 latency_gray_receiver camera_clock_time(.clk(cam_clk),.rst_n(cam_rst_n),
  .source_gray(diagnostic_time_gray),.time_us(camera_time));
 reg[31:0] sensor_frame_id,selected_frame_id,metadata_overruns;
 reg first_sent;
 wire first_selected_pixel=take_pixel&&(!active||!first_sent);
 wire metadata_busy,metadata_overrun;
 wire[64:0] received_metadata;
 always @(posedge cam_clk or negedge cam_rst_n)
 if(!cam_rst_n) begin sensor_frame_id<=0;selected_frame_id<=0;first_sent<=0;metadata_overruns<=0;end
 else begin
  if(sof) sensor_frame_id<=sensor_frame_id+1'b1;
  if(take_new) begin selected_frame_id<=sensor_frame_id+1'b1;first_sent<=0;end
  if(first_selected_pixel) first_sent<=1;
  if(metadata_overrun) metadata_overruns<=metadata_overruns+1'b1;
 end
 latency_mailbox #(.WIDTH(65)) capture_metadata_mailbox(
  .source_clk(cam_clk),.source_rst_n(cam_rst_n),.source_valid(first_selected_pixel),
  .source_data({take_new?req2:seen,take_new?(sensor_frame_id+1'b1):selected_frame_id,camera_time}),
  .source_busy(metadata_busy),.source_overrun(metadata_overrun),
  .destination_clk(clk),.destination_rst_n(rst_n),
  .destination_valid(diagnostic_tag_valid),.destination_data(received_metadata));
 assign diagnostic_tag_epoch=received_metadata[64];
 assign diagnostic_frame_id=received_metadata[63:32];
 assign diagnostic_first_us=received_metadata[31:0];
 counter_cdc metadata_overrun_counter(.source_clk(cam_clk),.source_rst_n(cam_rst_n),.source_binary(metadata_overruns),
  .destination_clk(clk),.destination_rst_n(rst_n),.destination_binary(diagnostic_overruns));

endmodule
