// Three immutable source slots and three result slots permit simultaneous
// capture, warp and display. Only a complete physical DDR write publishes a slot.
module video_frame_system_complete #(
 parameter IMG_WIDTH=1920,IMG_HEIGHT=1080,FIFO_ADDR_BITS=11,FISHEYE_MODEL=0,CLOCK_HZ=100000000,FINAL_OUTPUT_DELAY=5,TILE_WIDTH=64,TILE_HEIGHT=16,DETAIL_SUPPORT=0,
 parameter[31:0] SOURCE0_BASE=32'h00000000,SOURCE1_BASE=32'h00400000,SOURCE2_BASE=32'h00800000,
 parameter[31:0] OUTPUT0_BASE=32'h00c00000,OUTPUT1_BASE=32'h01000000,OUTPUT2_BASE=32'h01400000,
 parameter[31:0] MAP0_BASE=32'h01800000,MAP1_BASE=32'h02800000
)(
 input wire clk,rst_n,cam_clk,cam_rst_n,pix_clk,pix_rst_n,
 input wire cam_valid,input wire[15:0] cam_rgb565,input wire cam_sof,
 input wire cam_frame_done,cam_frame_drop,
 input wire video_hs,video_vs,video_de,input wire[15:0] video_x,video_y,
 output wire hdmi_hs,hdmi_vs,hdmi_de,output wire[15:0] hdmi_rgb565,output wire hdmi_observation,hdmi_raw,
 input wire output_detail_enable,
 input wire cfg_valid,input wire[5:0] cfg_addr,input wire[31:0] cfg_data,
 input wire cfg_commit,output wire cfg_ready,cfg_commit_ready,
 output wire[27:0] axi_awaddr,output wire[3:0] axi_awlen,
 output wire axi_awvalid,input wire axi_awready,
 output wire[255:0] axi_wdata,output wire[31:0] axi_wstrb,input wire axi_wready,axi_wlast,
 output wire[27:0] axi_araddr,output wire[3:0] axi_arlen,
 output wire axi_arvalid,input wire axi_arready,
 input wire[255:0] axi_rdata,input wire axi_rvalid,axi_rlast,
 output reg[31:0] captured_frames,processed_frames,repeated_frames,dropped_frames,
 output wire[31:0] display_underflow_lines,capture_overflow_frames,camera_skipped_frames,
 output reg error_sticky,output wire processing,output wire[15:0] config_version,
 output wire map_building,output wire[31:0] map_rebuild_cycles,
 output wire[31:0] map_invalid_pixels,map_domain_pixels,board_time_us,
 output wire[3:0] map_reject_flags,
 output wire[511:0] throughput_stats,
 output wire latency_valid,output wire[511:0] latency_sample
);

 wire[31:0] diagnostic_time_us,diagnostic_time_gray;
 wire[255:0] diagnostic_front_metadata;
 wire diagnostic_tag_valid,diagnostic_tag_epoch;
 wire[31:0] diagnostic_frame_id,diagnostic_first_us,diagnostic_capture_overrun,diagnostic_missing_capture_tags;
 latency_timebase #(.CLOCK_HZ(CLOCK_HZ)) diagnostic_timebase(
  .clk(clk),.rst_n(rst_n),.time_us(diagnostic_time_us),.time_gray(diagnostic_time_gray));
 assign board_time_us=diagnostic_time_us;

 localparam PIXELS=IMG_WIDTH*IMG_HEIGHT;
 localparam S_FREE=0,S_CAPTURE=1,S_READY=2,S_PROCESS=3;
 localparam O_FREE=0,O_WRITE=1,O_PENDING=2,O_FRONT=3;
 reg[1:0] source_state[0:2],output_state[0:2];reg[31:0] source_sequence[0:2];
 reg[1:0] capture_slot,engine_source_slot,engine_output_slot,front_slot,pending_slot;
 reg front_valid,pending_valid,engine_inflight,engine_start;
 reg[31:0] engine_source_base,engine_output_base;
 wire engine_busy,engine_done,engine_error;
 wire[31:0] engine_cycles;
 wire map_available,map_error,display_error,engine_pause;
 wire map_observation,map_raw,engine_source_tag_valid;
 wire[31:0] engine_source_id,engine_source_first_us;
 wire[31:0] map_base;wire signed[31:0] gain_q8,offset;
 wire[31:0] raw_display_underflow,raw_capture_overflow,raw_camera_skipped;
 counter_cdc display_stats(
  .source_clk(pix_clk),.source_rst_n(pix_rst_n),.source_binary(raw_display_underflow),
  .destination_clk(clk),.destination_rst_n(rst_n),.destination_binary(display_underflow_lines));
 counter_cdc overflow_stats(
  .source_clk(cam_clk),.source_rst_n(cam_rst_n),.source_binary(raw_capture_overflow),
  .destination_clk(clk),.destination_rst_n(rst_n),.destination_binary(capture_overflow_frames));
 counter_cdc skipped_stats(
  .source_clk(cam_clk),.source_rst_n(cam_rst_n),.source_binary(raw_camera_skipped),
  .destination_clk(clk),.destination_rst_n(rst_n),.destination_binary(camera_skipped_frames));
 function[31:0] source_address(input[1:0] slot);
  begin case(slot) 0:source_address=SOURCE0_BASE;1:source_address=SOURCE1_BASE;default:source_address=SOURCE2_BASE;endcase end
 endfunction
 function[31:0] output_address(input[1:0] slot);
  begin case(slot) 0:output_address=OUTPUT0_BASE;1:output_address=OUTPUT1_BASE;default:output_address=OUTPUT2_BASE;endcase end
 endfunction
 wire[31:0] front_base=output_address(front_slot);
 reg source_free_found,source_ready_found,output_free_found;
 reg[1:0] source_free_slot,source_ready_slot,output_free_slot;
 reg[31:0] newest_sequence;
 integer scan;
 always @* begin
  source_free_found=0;source_ready_found=0;output_free_found=0;
  source_free_slot=0;source_ready_slot=0;output_free_slot=0;newest_sequence=0;
  for(scan=0;scan<3;scan=scan+1) begin
   if(source_state[scan]==S_FREE&&!source_free_found) begin source_free_found=1;source_free_slot=scan;end
   if(source_state[scan]==S_READY&&(!source_ready_found||source_sequence[scan]>newest_sequence)) begin
    source_ready_found=1;source_ready_slot=scan;newest_sequence=source_sequence[scan];
   end
   if(output_state[scan]==O_FREE&&!output_free_found) begin output_free_found=1;output_free_slot=scan;end
  end
 end
 wire[3:0] cmd_valid,cmd_write,cmd_ready,w_valid,w_ready,r_valid,r_ready,r_last,mem_done,mem_error;
 wire[127:0] cmd_addr,w_strb;wire[15:0] cmd_len;wire[1023:0] w_data,r_data;
 wire display_event;
 video_display_burst_latency #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.FINAL_OUTPUT_DELAY(FINAL_OUTPUT_DELAY),.OBSERVATION_SUPPORT(1),.DETAIL_SUPPORT(DETAIL_SUPPORT)) display(
  .clk(clk),.rst_n(rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),
  .hs(video_hs),.vs(video_vs),.de(video_de),.x(video_x),.y(video_y),
  .front_valid(front_valid),.front_base(front_base),.frame_event(display_event),
  .cmd_valid(cmd_valid[0]),.cmd_write(cmd_write[0]),.cmd_addr(cmd_addr[31:0]),.cmd_len(cmd_len[3:0]),.cmd_ready(cmd_ready[0]),
  .r_valid(r_valid[0]),.r_data(r_data[255:0]),.r_last(r_last[0]),.r_ready(r_ready[0]),.done(mem_done[0]),.error(mem_error[0]),
  .out_hs(hdmi_hs),.out_vs(hdmi_vs),.out_de(hdmi_de),.out_rgb565(hdmi_rgb565),.out_observation(hdmi_observation),.out_raw(hdmi_raw),
  .output_detail_enable(output_detail_enable),
  .underflow_lines(raw_display_underflow),.error_sticky(display_error),
  .diagnostic_front_metadata(diagnostic_front_metadata),.diagnostic_time_gray(diagnostic_time_gray),
  .diagnostic_missing_capture_tags(diagnostic_missing_capture_tags),.diagnostic_capture_overrun(diagnostic_capture_overrun),
  .latency_valid(latency_valid),.latency_sample(latency_sample));
 assign w_valid[0]=0;assign w_data[255:0]=0;assign w_strb[31:0]=0;
 map_build_controller_complete #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.TILE_WIDTH(TILE_WIDTH),.TILE_HEIGHT(TILE_HEIGHT),.MAP0_BASE(MAP0_BASE),.MAP1_BASE(MAP1_BASE)) map_builder(
  .clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_addr(cfg_addr),.cfg_data(cfg_data),
  .cfg_commit(cfg_commit),.cfg_ready(cfg_ready),.cfg_commit_ready(cfg_commit_ready),
  .engine_busy(engine_inflight),.engine_start(engine_start),
  .map_available(map_available),.map_building(map_building),.engine_pause(engine_pause),.map_base(map_base),.config_version(config_version),
  .gain_q8(gain_q8),.offset(offset),.error_sticky(map_error),.rebuild_cycles(map_rebuild_cycles),.observation_active(map_observation),.raw_map_active(map_raw),
  .map_invalid_pixels(map_invalid_pixels),.map_domain_pixels(map_domain_pixels),.map_reject_flags(map_reject_flags),
  .cmd_valid(cmd_valid[2]),.cmd_ready(cmd_ready[2]),.cmd_write(cmd_write[2]),.cmd_addr(cmd_addr[95:64]),.cmd_len(cmd_len[11:8]),
  .w_valid(w_valid[2]),.w_ready(w_ready[2]),.w_data(w_data[767:512]),.w_strb(w_strb[95:64]),
  .mem_done(mem_done[2]),.mem_error(mem_error[2]));
 assign r_ready[2]=1'b1;
 fast_warp_engine #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.TILE_WIDTH(TILE_WIDTH),.TILE_HEIGHT(TILE_HEIGHT),.OBSERVATION_SUPPORT(1)) engine(
  .clk(clk),.rst_n(rst_n),.start(engine_start),.busy(engine_busy),.done(engine_done),.error(engine_error),
  .source_base(engine_source_base),.output_base(engine_output_base),.map_base(map_base),.gain_q8(gain_q8),.offset(offset),
  .observation_enable(map_observation),.source_tag_valid(engine_source_tag_valid),
  .source_frame_id(engine_source_id),.source_first_us(engine_source_first_us),.source_config_version(config_version),
  .cmd_valid(cmd_valid[3]),.cmd_ready(cmd_ready[3]),.cmd_write(cmd_write[3]),.cmd_addr(cmd_addr[127:96]),.cmd_len(cmd_len[15:12]),
  .w_valid(w_valid[3]),.w_ready(w_ready[3]),.w_data(w_data[1023:768]),.w_strb(w_strb[127:96]),
  .r_valid(r_valid[3]),.r_ready(r_ready[3]),.r_data(r_data[1023:768]),.r_last(r_last[3]),
  .mem_done(mem_done[3]),.mem_error(mem_error[3]),.cycles(engine_cycles),.tiles_completed(),.map_read_beats(),.source_read_beats());
 assign processing=engine_inflight;
 // Camera burst queue: assembling the next beat continues while a previous
 // command is loaded into the bridge or is physically executing in DDR.
 reg capture_request,capture_active,end_seen,end_good,capture_bad;
 wire cap_valid,cap_ready;wire[17:0] cap_data;
 video_capture_cdc_latency #(.FIFO_ADDR_BITS(FIFO_ADDR_BITS)) capture(
  .cam_clk(cam_clk),.cam_rst_n(cam_rst_n),.pix_valid(cam_valid),.rgb565(cam_rgb565),.sof(cam_sof),
  .frame_valid_done(cam_frame_done),.frame_drop(cam_frame_drop),.request_toggle(capture_request),
  .clk(clk),.rst_n(rst_n),.valid(cap_valid),.data(cap_data),.ready(cap_ready),
  .overflow_frames(raw_capture_overflow),.skipped_frames(raw_camera_skipped),
  .diagnostic_time_gray(diagnostic_time_gray),.diagnostic_tag_valid(diagnostic_tag_valid),
  .diagnostic_tag_epoch(diagnostic_tag_epoch),.diagnostic_frame_id(diagnostic_frame_id),
  .diagnostic_first_us(diagnostic_first_us),.diagnostic_overruns(diagnostic_capture_overrun));
 // Synchronous prefetch lets ADS infer DRM rather than distributed RAM. The
 // previous asynchronous queue mapped to RAM32 write ports with hold failures.
 (* syn_ramstyle="block_ram" *) reg[255:0] capture_queue[0:63];
 reg[255:0] capture_head,packed_pixels;
 reg[6:0] queue_write,queue_read;wire[6:0] queue_count=queue_write-queue_read;
 reg[3:0] pack_count;reg[31:0] pixel_count,written_words;
 localparam C_IDLE=0,C_CMD=1,C_FEED=2,C_WAIT=3;
 reg[1:0] writer_state;reg[4:0] batch_beats,send_index;
 assign cap_ready=capture_active&&!end_seen&&
  (cap_data[17:16]!=0 || pixel_count>=PIXELS || pack_count!=15 || queue_count<64);
 assign cmd_valid[1]=rst_n&&writer_state==C_CMD;
 assign cmd_write[1]=1'b1;
 assign cmd_addr[63:32]=source_address(capture_slot)+(written_words<<5);
 assign cmd_len[7:4]=batch_beats-1'b1;
 assign w_valid[1]=rst_n&&writer_state==C_FEED;
 assign w_data[511:256]=capture_head;
 assign w_strb[63:32]=32'hffffffff;assign r_ready[1]=1'b1;
 wire capture_pixel=cap_valid&&cap_ready&&cap_data[17:16]==0&&pixel_count<PIXELS;
 always @(posedge clk)
  if(rst_n&&capture_pixel&&pack_count==15)
   capture_queue[queue_write[5:0]]<={cap_data[15:0],packed_pixels[239:0]};
 wire[5:0] capture_read_address=queue_read[5:0]+(writer_state==C_FEED?6'd1:6'd0);
 always @(posedge clk)
  if(writer_state==C_CMD||(writer_state==C_FEED&&w_ready[1]))
   capture_head<=capture_queue[capture_read_address];
 wire capture_finished=capture_active&&end_seen&&writer_state==C_IDLE&&(queue_write==queue_read);
 wire capture_success=end_good&&!capture_bad&&pixel_count==PIXELS&&pack_count==0;
 wire launch_engine=!engine_inflight&&!engine_busy&&!engine_pause&&map_available&&source_ready_found&&output_free_found;
 integer slot,j;
 reg [1:0] older_count;
 always @* begin
  older_count=0;
  for(j=0;j<3;j=j+1) if(source_state[j]==S_READY&&j!=source_ready_slot) older_count=older_count+1'b1;
 end
 // At most two older ready slots plus one failed capture: fold the 0..3
 // increment before the single 32-bit accumulator, cycle-exact to both events.
 wire [1:0] dropped_increment=(launch_engine ? older_count : 2'd0)
                            +{1'b0,(capture_finished&&!capture_success)};
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  capture_request<=0;capture_active<=0;capture_slot<=0;end_seen<=0;end_good<=0;capture_bad<=0;
  queue_write<=0;queue_read<=0;packed_pixels<=0;pack_count<=0;pixel_count<=0;written_words<=0;
  writer_state<=C_IDLE;batch_beats<=0;send_index<=0;
  engine_start<=0;engine_inflight<=0;engine_source_slot<=0;engine_output_slot<=0;
  engine_source_base<=0;engine_output_base<=0;front_slot<=0;pending_slot<=0;front_valid<=0;pending_valid<=0;
  captured_frames<=0;processed_frames<=0;repeated_frames<=0;dropped_frames<=0;error_sticky<=0;
  for(slot=0;slot<3;slot=slot+1) begin source_state[slot]<=S_FREE;output_state[slot]<=O_FREE;source_sequence[slot]<=0;end
 end else begin
  engine_start<=0;
  if(dropped_increment!=0) dropped_frames<=dropped_frames+{30'd0,dropped_increment};
  if(map_error||display_error) error_sticky<=1;
  if(!capture_active&&source_free_found) begin
   capture_slot<=source_free_slot;source_state[source_free_slot]<=S_CAPTURE;
   capture_active<=1;capture_request<=~capture_request;end_seen<=0;end_good<=0;capture_bad<=0;
   queue_write<=0;queue_read<=0;packed_pixels<=0;pack_count<=0;pixel_count<=0;written_words<=0;
   writer_state<=C_IDLE;batch_beats<=0;send_index<=0;
  end
  if(cap_valid&&cap_ready) begin
   if(cap_data[17:16]==0) begin
    pixel_count<=pixel_count+1'b1;
    if(pixel_count<PIXELS) begin
     if(pack_count==15) begin queue_write<=queue_write+1'b1;packed_pixels<=0;pack_count<=0;end
     else begin packed_pixels[pack_count*16+:16]<=cap_data[15:0];pack_count<=pack_count+1'b1;end
    end else capture_bad<=1;
   end else begin end_seen<=1;end_good<=cap_data[17:16]==1;end
  end
  case(writer_state)
   C_IDLE: if(capture_active&&(queue_count>=16||(end_seen&&queue_count!=0))) begin
    batch_beats<=queue_count>=16?5'd16:queue_count[4:0];send_index<=0;writer_state<=C_CMD;
   end
   C_CMD: if(cmd_ready[1]) writer_state<=C_FEED;
   C_FEED: if(w_ready[1]) begin
    queue_read<=queue_read+1'b1;
    if(send_index+1==batch_beats) writer_state<=C_WAIT;
    else send_index<=send_index+1'b1;
   end
   C_WAIT: if(mem_done[1]) begin
    writer_state<=C_IDLE;written_words<=written_words+batch_beats;
    if(mem_error[1]) begin capture_bad<=1;error_sticky<=1;end
   end
  endcase
  if(capture_finished) begin
   capture_active<=0;
   if(capture_success) begin
    source_state[capture_slot]<=S_READY;source_sequence[capture_slot]<=captured_frames+1'b1;
    captured_frames<=captured_frames+1'b1;
   end else begin source_state[capture_slot]<=S_FREE;end
  end
  if(launch_engine) begin
   engine_source_slot<=source_ready_slot;source_state[source_ready_slot]<=S_PROCESS;
   engine_output_slot<=output_free_slot;output_state[output_free_slot]<=O_WRITE;
   engine_source_base<=source_address(source_ready_slot);engine_output_base<=output_address(output_free_slot);
   engine_start<=1;engine_inflight<=1;
   // Choose the newest complete camera frame and release older unprocessed ones.
   for(slot=0;slot<3;slot=slot+1)
    if(source_state[slot]==S_READY&&slot!=source_ready_slot) source_state[slot]<=S_FREE;
  end
  if(display_event) begin
   if(pending_valid) begin
    if(front_valid) output_state[front_slot]<=O_FREE;
    front_slot<=pending_slot;front_valid<=1;output_state[pending_slot]<=O_FRONT;pending_valid<=0;
   end else if(front_valid) repeated_frames<=repeated_frames+1'b1;
  end
  // start is one pulse into an idle engine. Ignore its previous sticky error on
  // that acceptance edge; a failed frame never becomes a displayed result.
  if(engine_inflight&&!engine_start&&(engine_done||(engine_error&&!engine_busy))) begin
   engine_inflight<=0;source_state[engine_source_slot]<=S_FREE;
   if(engine_error) begin output_state[engine_output_slot]<=O_FREE;error_sticky<=1;end
   else begin
    if(pending_valid&&!display_event) output_state[pending_slot]<=O_FREE;
    pending_slot<=engine_output_slot;pending_valid<=1;output_state[engine_output_slot]<=O_PENDING;
    processed_frames<=processed_frames+1'b1;
   end
  end
 end
 wire b_cmd_valid,b_cmd_ready,b_cmd_write,b_w_valid,b_w_ready,b_r_valid,b_r_ready,b_r_last,b_done,b_error;
 wire[31:0] b_cmd_addr,b_w_strb;wire[3:0] b_cmd_len;wire[255:0] b_w_data,b_r_data;
 throughput_diagnostics performance_counters(
  .clk(clk),.rst_n(rst_n),.engine_busy(engine_busy),.engine_inflight(engine_inflight),
  .engine_start(engine_start),.engine_done(engine_done),.engine_error(engine_error),.engine_cycles(engine_cycles),
  .camera_skipped_frames(camera_skipped_frames),.repeated_frames(repeated_frames),
  .map_available(map_available),.source_ready(source_ready_found),.output_free(output_free_found),
  .engine_cmd_valid(cmd_valid[3]),.engine_cmd_ready(cmd_ready[3]),
  .bridge_cmd_valid(b_cmd_valid),.bridge_cmd_ready(b_cmd_ready),.bridge_done(b_done),
  .axi_arvalid(axi_arvalid),.axi_arready(axi_arready),.axi_rvalid(axi_rvalid),.axi_rlast(axi_rlast),
  .axi_awvalid(axi_awvalid),.axi_awready(axi_awready),.axi_wready(axi_wready),.axi_wlast(axi_wlast),
  .counters(throughput_stats));
 ddr_burst_arbiter4_complete arbiter(
  .clk(clk),.rst_n(rst_n),.warp_inflight(engine_inflight),.cmd_valid(cmd_valid),.cmd_write(cmd_write),.cmd_ready(cmd_ready),.cmd_addr(cmd_addr),.cmd_len(cmd_len),
  .w_valid(w_valid),.w_ready(w_ready),.w_data(w_data),.w_strb(w_strb),.r_valid(r_valid),.r_ready(r_ready),.r_data(r_data),.r_last(r_last),.done(mem_done),.error(mem_error),
  .b_cmd_valid(b_cmd_valid),.b_cmd_ready(b_cmd_ready),.b_cmd_write(b_cmd_write),.b_cmd_addr(b_cmd_addr),.b_cmd_len(b_cmd_len),
  .b_w_valid(b_w_valid),.b_w_ready(b_w_ready),.b_w_data(b_w_data),.b_w_strb(b_w_strb),
  .b_r_valid(b_r_valid),.b_r_ready(b_r_ready),.b_r_data(b_r_data),.b_r_last(b_r_last),.b_done(b_done),.b_error(b_error));
 ddr_burst_bridge bridge(
  .clk(clk),.rst_n(rst_n),.cmd_valid(b_cmd_valid),.cmd_ready(b_cmd_ready),.cmd_write(b_cmd_write),.cmd_addr(b_cmd_addr),.cmd_len(b_cmd_len),
  .w_valid(b_w_valid),.w_ready(b_w_ready),.w_data(b_w_data),.w_strb(b_w_strb),
  .r_valid(b_r_valid),.r_ready(b_r_ready),.r_data(b_r_data),.r_last(b_r_last),.done(b_done),.error(b_error),
  .axi_awaddr(axi_awaddr),.axi_awlen(axi_awlen),.axi_awvalid(axi_awvalid),.axi_awready(axi_awready),
  .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wready(axi_wready),.axi_wlast(axi_wlast),
  .axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
  .axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid),.axi_rlast(axi_rlast));

 latency_frame_tracker #(.OBSERVATION_SUPPORT(1),.DETAIL_SUPPORT(DETAIL_SUPPORT)) diagnostic_tracker(
  .clk(clk),.rst_n(rst_n),.time_us(diagnostic_time_us),
  .capture_begin(!capture_active&&source_free_found),.capture_epoch(capture_request),.capture_slot(capture_slot),
  .capture_finished(capture_finished),.capture_success(capture_success),
  .capture_tag_valid(diagnostic_tag_valid),.capture_tag_epoch(diagnostic_tag_epoch),
  .capture_tag_id(diagnostic_frame_id),.capture_tag_time(diagnostic_first_us),
  .engine_start(engine_start),.engine_source_slot(engine_source_slot),.engine_output_slot(engine_output_slot),
  .engine_complete(engine_inflight&&!engine_start&&(engine_done||(engine_error&&!engine_busy))),
  .engine_error(engine_error),.config_version(config_version),.observation_active(map_observation),.raw_map_active(map_raw),.front_slot(front_slot),
  .front_metadata(diagnostic_front_metadata),.missing_capture_tags(diagnostic_missing_capture_tags),
  .engine_source_tag_valid(engine_source_tag_valid),.engine_source_id(engine_source_id),.engine_source_first_us(engine_source_first_us));

endmodule
