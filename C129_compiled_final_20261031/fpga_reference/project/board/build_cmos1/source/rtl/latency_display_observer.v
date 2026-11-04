// Observes actual SELECT and active-pixel edges. Never stalls display.
// FINAL_OUTPUT_DELAY=1 includes the board's one extra output register after
// video_display_burst's synchronous line-buffer read/out_de register.
module latency_display_observer #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,FINAL_OUTPUT_DELAY=1,DETAIL_SUPPORT=0
)(
 input wire clk,rst_n,input wire select_frame,new_core_epoch,
 input wire[255:0] front_metadata,input wire[31:0] time_gray,
 input wire pix_clk,pix_rst_n,new_pix_epoch,de,pixel_good,
 input wire[15:0] x,y,input wire[31:0] underflow_lines,
 input wire[31:0] missing_capture_tags,capture_metadata_overrun,
 output wire latency_valid,output wire[511:0] latency_sample,
 output wire selected_observation,selected_raw,
 input wire output_detail_enable
);
 reg[31:0] core_generation;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) core_generation<=0;
 else if(new_core_epoch) core_generation<=core_generation+1'b1;
 wire meta_busy,meta_overrun,meta_arrived;
 wire[287:0] meta_data;
 latency_mailbox #(.WIDTH(288)) selected_mailbox(
  .source_clk(clk),.source_rst_n(rst_n),.source_valid(select_frame),
  .source_data({core_generation,front_metadata}),.source_busy(meta_busy),.source_overrun(meta_overrun),
  .destination_clk(pix_clk),.destination_rst_n(pix_rst_n),.destination_valid(meta_arrived),.destination_data(meta_data));
 reg[31:0] core_overruns;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) core_overruns<=0;else if(meta_overrun) core_overruns<=core_overruns+1'b1;
 wire[31:0] pix_time;
 latency_gray_receiver pixel_clock_time(.clk(pix_clk),.rst_n(pix_rst_n),.source_gray(time_gray),.time_us(pix_time));
 wire first_now=de&&x==0&&y==0;
 wire last_now=de&&x==IMG_WIDTH-1&&y==IMG_HEIGHT-1;
 wire bad_now=de&&!pixel_good;
 wire first_event,last_event,bad_event,delay_overrun;
 // Bind the immutable result identity at its original first input pixel.
 // Only timestamps wait for the downstream line filter; a later mailbox
 // delivery cannot relabel pixels already travelling through that filter.
 wire frame_bind_event=(FINAL_OUTPUT_DELAY>32)?(first_now&&!new_pix_epoch):first_event;
 generate if(FINAL_OUTPUT_DELAY<=32)begin: short_output_delay
  localparam PIPE_WIDTH=(FINAL_OUTPUT_DELAY<1)?1:FINAL_OUTPUT_DELAY;
  reg[PIPE_WIDTH-1:0] first_pipe,last_pipe,bad_pipe;
  assign first_event=(FINAL_OUTPUT_DELAY==0)?first_now:first_pipe[PIPE_WIDTH-1];
  assign last_event=(FINAL_OUTPUT_DELAY==0)?last_now:last_pipe[PIPE_WIDTH-1];
  assign bad_event=(FINAL_OUTPUT_DELAY==0)?bad_now:bad_pipe[PIPE_WIDTH-1];
  assign delay_overrun=1'b0;
  integer p;
  always @(posedge pix_clk or negedge pix_rst_n)
  if(!pix_rst_n) begin first_pipe<=0;last_pipe<=0;bad_pipe<=0;end
  else begin
   first_pipe[0]<=first_now;last_pipe[0]<=last_now;bad_pipe[0]<=bad_now;
   for(p=1;p<PIPE_WIDTH;p=p+1)begin first_pipe[p]<=first_pipe[p-1];last_pipe[p]<=last_pipe[p-1];bad_pipe[p]<=bad_pipe[p-1];end
  end
 end else begin: line_output_delay
  // Two event counters replace three 1661-stage FF chains. Raster underflow
  // is a sticky property of the associated input frame, not a data pipeline.
  c129_latency_event_delay #(.DELAY(FINAL_OUTPUT_DELAY)) event_delay(
   .clk(pix_clk),.rst_n(pix_rst_n),.new_epoch(new_pix_epoch),
   .first_now(first_now),.last_now(last_now),.bad_now(bad_now),
   .first_event(first_event),.last_event(last_event),.bad_event(bad_event),.overrun(delay_overrun));
 end endgenerate
 reg[31:0] pix_generation,available_generation;
 reg available_metadata,active_frame,frame_bad,frame_missing,frame_repeat,last_id_valid,first_observed;
 reg[255:0] available_data,frame_data;
 reg[31:0] first_time,last_source_id,repeat_count,missing_count,pix_overruns;
 reg sample_send;reg[511:0] sample_data;
 wire report_busy,report_overrun;
 wire matching_metadata=available_metadata&&available_generation==pix_generation;
 assign selected_observation=matching_metadata&&available_data[84];
 // A missing result tag conservatively bypasses the experimental filter.
 assign selected_raw=!matching_metadata||available_data[86];
 function source_is_new(input[31:0] candidate,previous,input previous_valid);
  reg[31:0] difference;
  begin difference=candidate-previous;source_is_new=!previous_valid||(difference!=0&&!difference[31]);end
 endfunction
 wire available_id_new=source_is_new(available_data[63:32],last_source_id,last_id_valid);
 wire arriving_id_new=source_is_new(meta_data[63:32],last_source_id,last_id_valid);
 // Only monotonically incrementing diagnostic counters cross via Gray CDC.
 wire[31:0] capture_missing_pix,capture_overrun_pix,core_overrun_pix;
 counter_cdc missing_counter(.source_clk(clk),.source_rst_n(rst_n),.source_binary(missing_capture_tags),
  .destination_clk(pix_clk),.destination_rst_n(pix_rst_n),.destination_binary(capture_missing_pix));
 counter_cdc cam_overrun_counter(.source_clk(clk),.source_rst_n(rst_n),.source_binary(capture_metadata_overrun),
  .destination_clk(pix_clk),.destination_rst_n(pix_rst_n),.destination_binary(capture_overrun_pix));
 counter_cdc select_overrun_counter(.source_clk(clk),.source_rst_n(rst_n),.source_binary(core_overruns),
  .destination_clk(pix_clk),.destination_rst_n(pix_rst_n),.destination_binary(core_overrun_pix));
 reg[31:0] overrun_snapshot;
 wire[31:0] total_overruns=capture_overrun_pix+core_overrun_pix+pix_overruns;
 reg[15:0] report_flags;
 always @* begin
  report_flags=frame_data[95:80];
  if(frame_bad||bad_event) begin report_flags[0]=0;report_flags[1]=1;end
  if(frame_missing) begin report_flags[0]=0;report_flags[2]=1;end
  if(total_overruns!=overrun_snapshot) begin report_flags[0]=0;report_flags[3]=1;end
 end
 always @(posedge pix_clk or negedge pix_rst_n)
 if(!pix_rst_n) begin
  pix_generation<=0;available_generation<=0;available_metadata<=0;available_data<=0;
  active_frame<=0;frame_bad<=0;frame_missing<=0;frame_repeat<=0;frame_data<=0;first_observed<=0;
  first_time<=0;last_source_id<=0;last_id_valid<=0;repeat_count<=0;missing_count<=0;pix_overruns<=0;
  sample_send<=0;sample_data<=0;overrun_snapshot<=0;
 end else begin
  sample_send<=0;
  if(report_overrun||delay_overrun) pix_overruns<=pix_overruns+1'b1;
  if(new_pix_epoch) begin pix_generation<=pix_generation+1'b1;available_metadata<=0;active_frame<=0;first_observed<=0;end
  if(meta_arrived) begin
   available_metadata<=1;available_generation<=meta_data[287:256];available_data<=meta_data[255:0];
   // A late ID cannot turn a subsequent repeat into this frame's first output.
   // Conservatively remember a noncurrent generation too: bad/reset metadata
   // may suppress a sample, but can never invent a valid shorter latency.
   if(arriving_id_new&&(first_observed||frame_bind_event||meta_data[287:256]!=pix_generation)) begin
    last_source_id<=meta_data[63:32];last_id_valid<=1;
   end
  end
  if(frame_bind_event) begin
   first_observed<=1;frame_missing<=!matching_metadata;
   overrun_snapshot<=total_overruns;
   frame_repeat<=matching_metadata&&!available_id_new;
   if(matching_metadata) begin
    frame_data<=available_data;
    if(available_id_new) begin last_source_id<=available_data[63:32];last_id_valid<=1;end
    else repeat_count<=repeat_count+1'b1;
   end else begin
    // An invalid LAT1 record remains parseable, but its ID/times are unknown.
    frame_data<={32'b0,32'b0,32'b0,32'b0,32'd1000,16'h0004,16'b0,32'b0,32'h4c415431};
    missing_count<=missing_count+1'b1;
   end
  end
  if(first_event)begin
   active_frame<=1;first_time<=pix_time;frame_bad<=bad_event;
   // Bound to this frame's true final first pixel, not the UART request.
   frame_data[85]<=DETAIL_SUPPORT&&output_detail_enable;
  end else if(active_frame&&bad_event) frame_bad<=1;
  if(last_event&&active_frame) begin
   active_frame<=0;
   // A repeated scan is never reinterpreted as this source frame's first output.
   if(!frame_repeat) begin
    sample_data[255:0]<=frame_data;
    sample_data[95:80]<=report_flags;
    sample_data[287:256]<=first_time;
    sample_data[319:288]<=pix_time;
    sample_data[351:320]<=repeat_count;
    sample_data[383:352]<=underflow_lines;
    sample_data[415:384]<=capture_missing_pix;
    sample_data[447:416]<=missing_count;
    sample_data[479:448]<=total_overruns;
    sample_data[511:480]<=0;
    sample_send<=1;
   end
  end
 end
 latency_mailbox #(.WIDTH(512)) result_mailbox(
  .source_clk(pix_clk),.source_rst_n(pix_rst_n),.source_valid(sample_send),.source_data(sample_data),
  .source_busy(report_busy),.source_overrun(report_overrun),
  .destination_clk(clk),.destination_rst_n(rst_n),.destination_valid(latency_valid),.destination_data(latency_sample));
endmodule

// Fail-closed event delay for a fixed line-processing latency. A new input
// epoch cancels undelivered events so malformed short frames cannot relabel an
// old output pulse with later metadata. Normal 720p has 8250 porch clocks from
// its last active line to VS, comfortably exceeding C129's 1661-clock delay.
module c129_latency_event_delay #(parameter DELAY=1661)(
 input wire clk,rst_n,new_epoch,first_now,last_now,bad_now,
 output wire first_event,last_event,bad_event,output reg overrun
);
 function integer bits_for;input integer value;integer n;
  begin bits_for=1;for(n=value;n>1;n=n>>1)bits_for=bits_for+1;end
 endfunction
 localparam CW=bits_for(DELAY);
 reg[CW-1:0] first_count,last_count;
 reg first_pending,last_pending,input_active,input_bad,epoch_bad;
 wire collision=(first_now&&(first_pending||input_active))||
                (last_now&&last_pending)||(first_now&&last_now);
 assign first_event=first_pending&&first_count==1&&!new_epoch&&!epoch_bad&&!collision;
 assign last_event=last_pending&&last_count==1&&!new_epoch&&!epoch_bad&&!collision;
 assign bad_event=input_bad||((input_active||first_now)&&bad_now);
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  first_count<=0;last_count<=0;first_pending<=0;last_pending<=0;
  input_active<=0;input_bad<=0;epoch_bad<=0;overrun<=0;
 end else begin
  overrun<=0;
  if(new_epoch)begin
   if(first_pending||last_pending)overrun<=1;
   first_count<=0;last_count<=0;first_pending<=0;last_pending<=0;
   input_active<=0;input_bad<=0;epoch_bad<=0;
  end else if(collision)begin
   first_count<=0;last_count<=0;first_pending<=0;last_pending<=0;
   input_active<=0;epoch_bad<=1;overrun<=1;
  end else if(!epoch_bad)begin
   if(first_pending)begin
    if(first_count==1)begin first_count<=0;first_pending<=0;end
    else first_count<=first_count-1'b1;
   end
   if(last_pending)begin
    if(last_count==1)begin last_count<=0;last_pending<=0;end
    else last_count<=last_count-1'b1;
   end
   if(first_now)begin first_count<=DELAY;first_pending<=1;input_active<=1;input_bad<=bad_now;end
   if(last_now)begin last_count<=DELAY;last_pending<=1;input_active<=0;end
   if((input_active||first_now)&&bad_now)input_bad<=1;
  end
 end
endmodule
