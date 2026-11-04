// A request toggle selects exactly one complete sensor frame. The sensor never stalls.
// Packet bits17:16: 00=pixel, 01=validated frame end, 10=bad frame end.
module video_capture_cdc #(parameter FIFO_ADDR_BITS=11)(
 input wire cam_clk,cam_rst_n,pix_valid,input wire[15:0] rgb565,
 input wire sof,frame_valid_done,frame_drop,input wire request_toggle,
 input wire clk,rst_n,output wire valid,output wire[17:0] data,input wire ready,
 output reg[31:0] overflow_frames,output reg[31:0] skipped_frames
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
endmodule
