// Background FPGA map construction. The published bank is immutable until
// processing drains; a failed replacement never discards the working map.
// Transactions contain the original 18 words plus H[0..8] at 0x20..0x28.
module map_build_controller_complete #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,TILE_WIDTH=64,TILE_HEIGHT=16,
 parameter [31:0] MAP0_BASE=32'h01800000,MAP1_BASE=32'h02800000
)(
 input wire clk,rst_n,
 input wire cfg_valid,input wire[5:0] cfg_addr,input wire[31:0] cfg_data,
 input wire cfg_commit,output wire cfg_ready,cfg_commit_ready,
 input wire engine_busy,engine_start,
 output reg map_available,output wire map_building,engine_pause,
 output reg[31:0] map_base,output reg[15:0] config_version,
 output reg signed[31:0] gain_q8,offset,
 output reg error_sticky,output reg[31:0] rebuild_cycles,output reg observation_active,raw_map_active,
 output reg[31:0] map_invalid_pixels,map_domain_pixels,
 output reg[3:0] map_reject_flags,
 output wire cmd_valid,input wire cmd_ready,output wire cmd_write,
 output wire[31:0] cmd_addr,output wire[3:0] cmd_len,
 output wire w_valid,input wire w_ready,output wire[255:0] w_data,
 output wire[31:0] w_strb,input wire mem_done,mem_error
);
 localparam IDLE=0,CHECK=1,START=2,COLLECT=3,COMMAND=4,
            WRITE=5,WAIT_DONE=6,DRAIN=7,PUBLISH=8;
 reg[3:0] state;
 reg[31:0] shadow[0:26];
 reg[26:0] written;
 reg transaction_open;
 reg[4:0] expected_index;
 // Evaluate wide mode/tag validation when their ordered word is written.
 // The result is available with shadow[] on the next cycle, so COMMIT keeps
 // exactly the same handshake timing without a 32-bit compare on state CE.
 (* syn_preserve=1 *) reg mode_word_valid,tag_word_valid;
 reg[31:0] bank0_geometry[0:24],bank1_geometry[0:24];
 reg[1:0] cache_valid,quiet;
 reg bank,build_bank,target_bank,build_failed,newly_built;
 reg[31:0] bank_invalid_pixels[0:1],bank_domain_pixels[0:1];
 reg[4:0] compare_index;
 reg match0,match1,mapper_reset_n,compare_phase;
 reg[31:0] compare_shadow_q,compare_bank0_q,compare_bank1_q;
 wire[4:0] compare_address=compare_index<14?compare_index:compare_index+5'd2;
 wire word_match0=compare_shadow_q==compare_bank0_q;
 wire word_match1=compare_shadow_q==compare_bank1_q;
 wire cfg_address_ok=cfg_addr<18||(cfg_addr>=6'h20&&cfg_addr<=6'h28);
 wire[5:0] expected_addr = (expected_index < 5'd18) ? {1'b0,expected_index} : ({1'b0,expected_index}+6'd14);
 assign cfg_ready=rst_n&&state==IDLE&&!cfg_commit;
 assign cfg_commit_ready=rst_n&&state==IDLE&&transaction_open&&expected_index==27&&(&written)&&
   tag_word_valid&&mode_word_valid;
 assign map_building=state!=IDLE;
 // Processing and old-map reads continue during CHECK and generation.
 // This is also high in PUBLISH, so no engine start can race publication.
 assign engine_pause=state==DRAIN||state==PUBLISH;
 reg[255:0] pack_data;
 reg[1:0] lane;
 reg[4:0] send_index,batch_beats;
 // Producer and physical writer run independently. A complete coordinate
 // beat is never overwritten until copied into the bridge's own buffer.
 localparam W_IDLE=0,W_COMMAND=1,W_WRITE=2,W_WAIT=3;
 reg[1:0] writer_state;
 reg[6:0] queue_write,queue_read;
 wire[6:0] queue_count=queue_write-queue_read;
 reg stream_done;
 (* syn_ramstyle="block_ram" *) reg[255:0] batch[0:63];
 reg[255:0] batch_q;
 // Select the address before one synchronous RAM read. Separate conditional
 // reads infer duplicate RAMs and a wide data mux in the production compiler.
 wire batch_read_enable=state==COLLECT&&
   (writer_state==W_COMMAND||(writer_state==W_WRITE&&w_ready));
 wire[5:0] batch_read_address=writer_state==W_COMMAND?
   queue_read[5:0]:queue_read[5:0]+6'd1;
 reg[31:0] write_offset;
 wire mapper_valid,mapper_last,mapper_busy,mapper_done,mapper_error,mapper_domain_error;
 wire[38:0] mapper_entry;
 reg[10:0] observation_x,observation_y;
 wire observation_build=(IMG_WIDTH==1280&&IMG_HEIGHT==720)&&shadow[16][1];
 wire observation_pixel=observation_build&&observation_x<320&&observation_y<192;
 wire observation_image=observation_pixel&&observation_y<180;
 wire[38:0] guarded_entry=observation_pixel?
   // Center of each 4x4 source cell: (4*u+1.5,4*v+1.5). The existing
   // bilinear sampler averages its central 2x2 pixels; max tile ROI=256x62.
   (observation_image?{1'b1,observation_x[8:0],2'b01,8'h80,observation_y[8:0],2'b01,8'h80}:39'b0):mapper_entry;
 wire mapper_ready=state==COLLECT&&!stream_done&&((lane!=3&&!mapper_last)||queue_count<64);
 wire mapper_accept=mapper_valid&&mapper_ready;
 reg[255:0] completed_pack;
 always @* begin
  completed_pack=pack_data;
  completed_pack[lane*64+:64]={25'b0,guarded_entry};
 end
 wire[31:0] guard_invalid_pixels,guard_domain_pixels;
 wire[3:0] guard_reject_flags;
 // Isolate the source-window coordinate mux from the guard's neighbor and
 // tile-bound merge path. Packing/DDR and producer backpressure stay on the
 // original acceptance edge; only the safety inspection follows one cycle.
 reg[38:0] guard_input_entry;
 reg guard_input_valid,guard_input_last,guard_input_domain,guard_input_observation;
 wire guard_internal_pending;
 wire guard_checks_pending=guard_input_valid||guard_internal_pending;
 always @(posedge clk) if(mapper_accept) begin
  guard_input_entry<=guarded_entry;
  guard_input_last<=mapper_last;
  guard_input_domain<=mapper_domain_error&&!observation_pixel;
  guard_input_observation<=observation_pixel;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n) guard_input_valid<=0;
 else if(state!=COLLECT) guard_input_valid<=0;
 else guard_input_valid<=mapper_accept;
 complete_map_safety_guard #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
  .TILE_WIDTH(TILE_WIDTH),.TILE_HEIGHT(TILE_HEIGHT),.SOURCE_WINDOW_SUPPORT(1)) map_guard(
  .clk(clk),.rst_n(rst_n),.start(state==START),.s_valid(guard_input_valid&&state==COLLECT),
  .s_entry(guard_input_entry),.s_last(guard_input_last),.s_domain_error(guard_input_domain),
  .s_observation(guard_input_observation),
  .invalid_pixels(guard_invalid_pixels),.domain_pixels(guard_domain_pixels),
  .reject_flags(guard_reject_flags),.checks_pending(guard_internal_pending));
 wire[287:0] projective={shadow[26],shadow[25],shadow[24],shadow[23],shadow[22],
                         shadow[21],shadow[20],shadow[19],shadow[18]};
 complete_fisheye_mapper #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) mapper(
  .clk(clk),.rst_n(rst_n&&mapper_reset_n),.start(state==START),
  .cfg_fx_src(shadow[0]),.cfg_fy_src(shadow[1]),.cfg_cx_src(shadow[2]),.cfg_cy_src(shadow[3]),
  .cfg_cx_out(shadow[4]),.cfg_cy_out(shadow[5]),.cfg_fx_out(shadow[6]),.cfg_fy_out(shadow[7]),
  .cfg_k1(shadow[8]),.cfg_k2(shadow[9]),.cfg_k3(shadow[10]),.cfg_k4(shadow[11]),
  .cfg_cos(shadow[12]),.cfg_sin(shadow[13]),.cfg_mode({31'b0,shadow[16][0]}),
  .cfg_projective(projective),.map_ready(mapper_ready),
  .busy(mapper_busy),.done(mapper_done),.map_valid(mapper_valid),
  .map_entry(mapper_entry),.map_last(mapper_last),.map_domain_error(mapper_domain_error),.error(mapper_error));
 assign cmd_valid=state==COLLECT&&writer_state==W_COMMAND;
 assign cmd_write=1'b1;
 assign cmd_addr=(build_bank?MAP1_BASE:MAP0_BASE)+write_offset;
 assign cmd_len=batch_beats-1'b1;
 assign w_valid=state==COLLECT&&writer_state==W_WRITE;
 assign w_data=batch_q;
 assign w_strb=32'hffffffff;
 always @(posedge clk) begin
  if(mapper_accept&&(lane==3||mapper_last)) batch[queue_write[5:0]]<=completed_pack;
  if(batch_read_enable) batch_q<=batch[batch_read_address];
 end
 integer i;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=IDLE;written<=0;transaction_open<=0;expected_index<=0;mode_word_valid<=0;tag_word_valid<=0;cache_valid<=0;quiet<=0;bank<=1;build_bank<=0;target_bank<=0;
  map_available<=0;map_base<=MAP0_BASE;config_version<=0;gain_q8<=256;offset<=0;observation_active<=0;raw_map_active<=0;
  observation_x<=0;observation_y<=0;
  error_sticky<=0;rebuild_cycles<=0;compare_index<=0;compare_phase<=0;match0<=0;match1<=0;
  map_invalid_pixels<=0;map_domain_pixels<=0;map_reject_flags<=0;
  bank_invalid_pixels[0]<=0;bank_invalid_pixels[1]<=0;bank_domain_pixels[0]<=0;bank_domain_pixels[1]<=0;
  compare_shadow_q<=0;compare_bank0_q<=0;compare_bank1_q<=0;
  mapper_reset_n<=0;pack_data<=0;lane<=0;send_index<=0;batch_beats<=0;
  write_offset<=0;build_failed<=0;newly_built<=0;
  writer_state<=W_IDLE;queue_write<=0;queue_read<=0;stream_done<=0;
  for(i=0;i<27;i=i+1) shadow[i]<=0;
 end else begin
  // Preserve the FPGA radial/reciprocal tables between successful starts.
  mapper_reset_n<=1;
  if(state==START)begin observation_x<=0;observation_y<=0;end
  else if(mapper_accept)begin
   if(observation_x==IMG_WIDTH-1)begin observation_x<=0;observation_y<=observation_y+1'b1;end
   else observation_x<=observation_x+1'b1;
  end
  if(map_building) rebuild_cycles<=rebuild_cycles+1'b1;
  if(state==COLLECT&&mapper_error) begin
   build_failed<=1;error_sticky<=1;
  end
  if(cfg_valid&&cfg_ready) begin
   // UART FE/COMP opens a fresh transaction. Any serial abort is the same
   // sentinel address with data=0. Neither is an ordinary geometry register.
   if(cfg_addr==6'h3f) begin
    written<=0;expected_index<=0;transaction_open<=cfg_data==32'h434f4d50;
    mode_word_valid<=0;tag_word_valid<=0;
   end else if(transaction_open) begin
    // Ordered addressing also detects an entirely missing/duplicated packet,
    // including a lost BEGIN followed by the start of a new transaction.
    if(expected_index<5'd27&&cfg_addr==expected_addr) begin
     shadow[expected_index]<=cfg_data;written[expected_index]<=1;expected_index<=expected_index+1'b1;
     if(expected_index==16) mode_word_valid<=cfg_data==0||cfg_data==1||
       (IMG_WIDTH==1280&&IMG_HEIGHT==720&&cfg_data==3);
     if(expected_index==17) tag_word_valid<=cfg_data==32'h46495348;
    end else if(cfg_address_ok) begin written<=0;expected_index<=0;transaction_open<=0;end
   end
  end
  case(state)
   IDLE: begin
    if(cfg_commit&&cfg_commit_ready) begin
     written<=0;transaction_open<=0;expected_index<=0;build_bank<=~bank;target_bank<=~bank;
     compare_index<=0;compare_phase<=0;match0<=cache_valid[0];match1<=cache_valid[1];state<=CHECK;
     pack_data<=0;lane<=0;send_index<=0;write_offset<=0;
     build_failed<=0;newly_built<=0;rebuild_cycles<=0;map_reject_flags<=0;
     writer_state<=W_IDLE;queue_write<=0;queue_read<=0;stream_done<=0;
    end
   end
   CHECK: begin
    // Separate the large key mux from equality/publication control. This
    // removes the compare_index -> wide mux -> equality -> state/quiet path.
    if(!compare_phase) begin
     compare_shadow_q<=shadow[compare_address];
     compare_bank0_q<=bank0_geometry[compare_index];
     compare_bank1_q<=bank1_geometry[compare_index];compare_phase<=1;
    end else if(compare_index==24) begin
     if((match0&&word_match0)||(match1&&word_match1)) begin
      target_bank<=!(match0&&word_match0);quiet<=0;state<=DRAIN;
     end else begin
      cache_valid[build_bank]<=0;mapper_reset_n<=1;state<=START;
     end
    end else begin
     match0<=match0&&word_match0;match1<=match1&&word_match1;
     compare_index<=compare_index+1'b1;compare_phase<=0;
    end
   end
   START: state<=COLLECT;
   COLLECT: begin
    if(mapper_accept) begin
     if(lane==3||mapper_last) begin
      lane<=0;pack_data<=0;queue_write<=queue_write+1'b1;
     end else begin pack_data<=completed_pack;lane<=lane+1'b1;end
     if(mapper_last) stream_done<=1;
    end
    case(writer_state)
     W_IDLE: if(queue_count>=16||(stream_done&&queue_count!=0)) begin
      batch_beats<=queue_count>=16?5'd16:queue_count[4:0];send_index<=0;writer_state<=W_COMMAND;
     end
     W_COMMAND: if(cmd_ready) writer_state<=W_WRITE;
     W_WRITE: if(w_ready) begin
      queue_read<=queue_read+1'b1;
      if(send_index+1==batch_beats) writer_state<=W_WAIT;
      else send_index<=send_index+1'b1;
     end
     W_WAIT: if(mem_done) begin
      writer_state<=W_IDLE;write_offset<=write_offset+{22'b0,batch_beats,5'b0};
      if(mem_error) begin
       error_sticky<=1;mapper_reset_n<=0;state<=IDLE;
       map_invalid_pixels<=guard_invalid_pixels;map_domain_pixels<=guard_domain_pixels;
       map_reject_flags<=guard_reject_flags|4'b1000|{3'b0,build_failed||mapper_error};
      end
     end
    endcase
    // Counts/guard decisions include the final accepted coordinate, and the
    // bank is not eligible until its final physical DDR completion.
    if(stream_done&&writer_state==W_IDLE&&queue_count==0&&!guard_checks_pending) begin
     map_invalid_pixels<=guard_invalid_pixels;map_domain_pixels<=guard_domain_pixels;
     map_reject_flags<=guard_reject_flags|{3'b0,build_failed||mapper_error};
     if(!build_failed&&!mapper_error&&guard_reject_flags==0) begin
      newly_built<=1;target_bank<=build_bank;quiet<=0;state<=DRAIN;
     end else begin
      // ROI and all-black results are recoverable pose policy rejections.
      // Keep the old bank usable without poisoning the system's sticky fault;
      // the transaction rejection flags remain visible to the host.
      if(build_failed||mapper_error||guard_reject_flags[0]) error_sticky<=1;
      state<=IDLE;
     end
    end
   end
   DRAIN: begin
    if(engine_busy||engine_start) quiet<=0;
    else if(quiet==2) state<=PUBLISH;
    else quiet<=quiet+1'b1;
   end
   PUBLISH: begin
    // Protection against an integration error: a late start postpones commit.
    if(engine_busy||engine_start) begin quiet<=0;state<=DRAIN;end
    else begin
     bank<=target_bank;map_base<=target_bank?MAP1_BASE:MAP0_BASE;
     gain_q8<=shadow[14];offset<=shadow[15];config_version<=config_version+1'b1;
     observation_active<=shadow[16][1];
     raw_map_active<=!shadow[16][0];
     map_available<=1;state<=IDLE;
     if(newly_built) begin
      cache_valid[target_bank]<=1;
      bank_invalid_pixels[target_bank]<=map_invalid_pixels;
      bank_domain_pixels[target_bank]<=map_domain_pixels;
      for(i=0;i<25;i=i+1) begin
       if(target_bank) bank1_geometry[i]<=shadow[i<14?i:i+2];
       else bank0_geometry[i]<=shadow[i<14?i:i+2];
      end
     end else begin
      map_invalid_pixels<=bank_invalid_pixels[target_bank];
      map_domain_pixels<=bank_domain_pixels[target_bank];map_reject_flags<=0;
     end
    end
   end
   default: begin mapper_reset_n<=0;state<=IDLE;error_sticky<=1;end
  endcase
 end
endmodule

// Inspect the FPGA-produced raster stream before its bank can be published.
// A per-column synchronous RAM retains bounds across the 16 rows of a tile.
// Only the current 64-pixel segment uses registers. Every tile's first row
// ignores RAM, so reset/start never needs to clear a memory asynchronously.
// Domain-invalid pixels are explicit black entries and counted separately;
// malformed coordinates, unsupported source ROI and a completely black frame
// are fatal. No map coordinate, source ROI or sample is silently clipped.
module complete_map_safety_guard #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,TILE_WIDTH=64,TILE_HEIGHT=16,
 parameter MAX_ROI_WIDTH=256,MAX_ROI_HEIGHT=64,SOURCE_WINDOW_SUPPORT=0
)(
 input wire clk,rst_n,start,s_valid,
 input wire[38:0] s_entry,input wire s_last,s_domain_error,
 input wire s_observation,
 output reg[31:0] invalid_pixels,domain_pixels,
 output reg[3:0] reject_flags,
 output wire checks_pending
);
 localparam TILE_COLUMNS=(IMG_WIDTH+TILE_WIDTH-1)/TILE_WIDTH;
 localparam COL_BITS=TILE_COLUMNS<2?1:$clog2(TILE_COLUMNS);
 reg[15:0] x,y;
 reg[COL_BITS-1:0] column;
 reg[15:0] segment_x,tile_y;
 reg seen_valid,ended;
 reg roi_check_pending;
 assign checks_pending=roi_check_pending;
 // {any, min_x, min_y, max_x_including_neighbor, max_y_including_neighbor}
 (* syn_ramstyle="block_ram" *) reg[44:0] bounds[0:TILE_COLUMNS-1];
 reg[44:0] bounds_q,current_bounds;
 wire[COL_BITS-1:0] next_column=column==TILE_COLUMNS-1?0:column+1'b1;
 // Prefetch the next column while processing the current segment. Holding
 // segment_x==0 also permits an arbitrarily stalled first pixel safely.
 wire[COL_BITS-1:0] read_column=segment_x==0?column:next_column;
 wire segment_last=segment_x==TILE_WIDTH-1||x==IMG_WIDTH-1;
 wire tile_last=tile_y==TILE_HEIGHT-1||y==IMG_HEIGHT-1;
 wire[44:0] prior_bounds=segment_x==0?(tile_y==0?45'b0:bounds_q):current_bounds;
 wire[10:0] sample_x=s_entry[37:27],sample_y=s_entry[18:8];
 wire malformed=s_entry[38]&&(sample_x>=IMG_WIDTH||sample_y>=IMG_HEIGHT);
 wire sample_inside=s_entry[38]&&!malformed;
 wire main_sample_inside=sample_inside&&(!SOURCE_WINDOW_SUPPORT||!s_observation);
 wire[10:0] sample_max_x=sample_x<IMG_WIDTH-1&&s_entry[26:19]!=0?sample_x+1'b1:sample_x;
 wire[10:0] sample_max_y=sample_y<IMG_HEIGHT-1&&s_entry[7:0]!=0?sample_y+1'b1:sample_y;
 wire prior_any=prior_bounds[44];
 wire[10:0] prior_min_x=prior_bounds[43:33],prior_min_y=prior_bounds[32:22];
 wire[10:0] prior_max_x=prior_bounds[21:11],prior_max_y=prior_bounds[10:0];
 wire[10:0] merged_min_x=!prior_any||sample_x<prior_min_x?sample_x:prior_min_x;
 wire[10:0] merged_min_y=!prior_any||sample_y<prior_min_y?sample_y:prior_min_y;
 wire[10:0] merged_max_x=!prior_any||sample_max_x>prior_max_x?sample_max_x:prior_max_x;
 wire[10:0] merged_max_y=!prior_any||sample_max_y>prior_max_y?sample_max_y:prior_max_y;
 wire[44:0] merged_bounds=sample_inside?{1'b1,merged_min_x,merged_min_y,merged_max_x,merged_max_y}:prior_bounds;
 // current_bounds already captures the merged final sample at segment end.
 // Check its ROI one cycle later, breaking neighbor/merge/alignment/compare
 // into two paths without another copy of the 45-bit bounds. The engine
 // fetches complete 16-pixel RGB565 beats, including left/right padding.
 wire[11:0] aligned_left={1'b0,current_bounds[43:37],4'b0};
 wire[11:0] aligned_right=({1'b0,current_bounds[21:11]}+12'd16)&12'hff0;
 wire[11:0] aligned_width=aligned_right-aligned_left;
 wire[11:0] bounded_height={1'b0,current_bounds[10:0]}-{1'b0,current_bounds[32:22]}+12'd1;
 wire roi_bad=current_bounds[44]&&(aligned_width>MAX_ROI_WIDTH||bounded_height>MAX_ROI_HEIGHT);
 always @(posedge clk) begin
  bounds_q<=bounds[read_column];
  if(rst_n&&!start&&s_valid&&!ended&&segment_last) bounds[column]<=merged_bounds;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  x<=0;y<=0;column<=0;segment_x<=0;tile_y<=0;seen_valid<=0;ended<=0;
  current_bounds<=0;invalid_pixels<=0;domain_pixels<=0;reject_flags<=0;roi_check_pending<=0;
 end else if(start) begin
  x<=0;y<=0;column<=0;segment_x<=0;tile_y<=0;seen_valid<=0;ended<=0;
  current_bounds<=0;invalid_pixels<=0;domain_pixels<=0;reject_flags<=0;roi_check_pending<=0;
 end else begin
  roi_check_pending<=s_valid&&!ended&&segment_last&&tile_last;
  // This runs even if the producer pauses or ends after the final pixel.
  if(roi_check_pending&&roi_bad) reject_flags[1]<=1;
  if(s_valid) begin
  if(ended) reject_flags[0]<=1;
  else begin
   current_bounds<=merged_bounds;
   if(!s_entry[38]) invalid_pixels<=invalid_pixels+1'b1;
   if(s_domain_error) domain_pixels<=domain_pixels+1'b1;
   // The raw source window cannot make an entirely invalid main view pass.
   if(main_sample_inside)seen_valid<=1;
   if(malformed||s_last!=(x==IMG_WIDTH-1&&y==IMG_HEIGHT-1)) reject_flags[0]<=1;
   if(s_last) begin
    ended<=1;
    if(!seen_valid&&!main_sample_inside) reject_flags[2]<=1;
   end
   if(x==IMG_WIDTH-1) begin
    x<=0;y<=y+1'b1;column<=0;segment_x<=0;
    tile_y<=tile_y==TILE_HEIGHT-1?0:tile_y+1'b1;
   end else begin
    x<=x+1'b1;
    if(segment_last) begin column<=column+1'b1;segment_x<=0;end
    else segment_x<=segment_x+1'b1;
   end
  end
  end
 end
endmodule
