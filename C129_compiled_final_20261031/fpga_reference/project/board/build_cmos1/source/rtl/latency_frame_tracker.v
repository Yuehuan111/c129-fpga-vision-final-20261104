// Metadata-only observer: no signal from this module controls the video path.
module latency_frame_tracker #(parameter OBSERVATION_SUPPORT=0,DETAIL_SUPPORT=0)(
 input wire clk,rst_n,input wire[31:0] time_us,
 input wire capture_begin,capture_epoch,input wire[1:0] capture_slot,
 input wire capture_finished,capture_success,
 input wire capture_tag_valid,capture_tag_epoch,
 input wire[31:0] capture_tag_id,capture_tag_time,
 input wire engine_start,input wire[1:0] engine_source_slot,engine_output_slot,
 input wire engine_complete,engine_error,input wire[15:0] config_version,
 input wire observation_active,
 input wire raw_map_active,
 input wire[1:0] front_slot,
 output wire[255:0] front_metadata,output reg[31:0] missing_capture_tags,
 output wire engine_source_tag_valid,output wire[31:0] engine_source_id,engine_source_first_us
);
 reg pending_valid;reg pending_epoch;reg[31:0] pending_id,pending_time;
 // Diagnostics only: break capture-success decode from the 32-bit counter.
 // Source validity, IDs and all metadata timestamps still use this cycle.
 reg missing_capture_event;
 reg source_valid[0:2];reg[31:0] source_id[0:2],source_first[0:2],source_done[0:2];
 reg[255:0] output_metadata[0:2];
 integer i;
 assign front_metadata=output_metadata[front_slot];
 assign engine_source_tag_valid=source_valid[engine_source_slot];
 assign engine_source_id=source_id[engine_source_slot];
 assign engine_source_first_us=source_first[engine_source_slot];
 wire tag_now=capture_tag_valid&&capture_tag_epoch==capture_epoch;
 wire tag_available=tag_now||(pending_valid&&pending_epoch==capture_epoch);
 wire[31:0] chosen_id=tag_now?capture_tag_id:pending_id;
 wire[31:0] chosen_time=tag_now?capture_tag_time:pending_time;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  pending_valid<=0;pending_epoch<=0;pending_id<=0;pending_time<=0;missing_capture_tags<=0;missing_capture_event<=0;
  for(i=0;i<3;i=i+1) begin
   source_valid[i]<=0;source_id[i]<=0;source_first[i]<=0;source_done[i]<=0;output_metadata[i]<=0;
  end
 end else begin
  missing_capture_event<=capture_finished&&capture_success&&!tag_available;
  if(missing_capture_event) missing_capture_tags<=missing_capture_tags+1'b1;
  if(capture_begin) pending_valid<=0;
  if(tag_now) begin pending_valid<=1;pending_epoch<=capture_tag_epoch;pending_id<=capture_tag_id;pending_time<=capture_tag_time;end
  if(capture_finished&&capture_success) begin
   source_valid[capture_slot]<=tag_available;source_id[capture_slot]<=chosen_id;
   source_first[capture_slot]<=chosen_time;source_done[capture_slot]<=time_us;
  end
  if(engine_start) begin
   output_metadata[engine_output_slot][31:0]<=32'h4c415431;
   output_metadata[engine_output_slot][63:32]<=source_id[engine_source_slot];
   output_metadata[engine_output_slot][95:64]<={
    (source_valid[engine_source_slot]?16'h0001:16'h0004)|((OBSERVATION_SUPPORT&&observation_active)?16'h0010:16'b0)|
    ((DETAIL_SUPPORT&&raw_map_active)?16'h0040:16'b0),config_version};
   output_metadata[engine_output_slot][127:96]<=32'd1000;
   output_metadata[engine_output_slot][159:128]<=source_first[engine_source_slot];
   output_metadata[engine_output_slot][191:160]<=source_done[engine_source_slot];
   output_metadata[engine_output_slot][223:192]<=time_us;
   output_metadata[engine_output_slot][255:224]<=0;
  end
  if(engine_complete) begin
   output_metadata[engine_output_slot][255:224]<=time_us;
   if(engine_error) output_metadata[engine_output_slot][80]<=0;
  end
 end
endmodule
