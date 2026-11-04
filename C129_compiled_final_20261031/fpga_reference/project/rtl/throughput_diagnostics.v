`timescale 1ns/1ps
// Passive core-clock observation only: no outputs feed the datapath.
// All counters naturally wrap at 2^32; compare successive board-second samples.
// last_frame_cycles latches the engine's own counter on successful done.
module throughput_diagnostics(
 input wire clk,rst_n,engine_busy,engine_inflight,engine_start,engine_done,engine_error,
 input wire[31:0] engine_cycles,camera_skipped_frames,repeated_frames,
 input wire map_available,source_ready,output_free,
 input wire engine_cmd_valid,engine_cmd_ready,
 input wire bridge_cmd_valid,bridge_cmd_ready,bridge_done,
 input wire axi_arvalid,axi_arready,axi_rvalid,axi_rlast,
 input wire axi_awvalid,axi_awready,axi_wready,axi_wlast,
 output wire[511:0] counters
);
 reg[31:0] last_frame_cycles,engine_busy_cycles;
 reg[31:0] idle_no_source_cycles,idle_no_output_cycles,idle_map_wait_cycles;
 reg[31:0] engine_cmd_wait_cycles,engine_memory_cycles,memory_active_cycles;
 reg[31:0] ddr_cmd_wait_cycles,ddr_read_first_wait_cycles,ddr_read_gap_cycles;
 reg[31:0] ddr_write_wait_cycles,ddr_read_bursts,ddr_data_beats;
 reg memory_active,memory_engine,read_active,read_seen_first,write_active;
 wire read_start=axi_arvalid&&axi_arready;
 wire write_start=axi_awvalid&&axi_awready;
 wire read_pending=read_active||read_start;
 wire write_pending=write_active||write_start;
 wire read_beat=read_pending&&axi_rvalid;
 wire write_beat=write_pending&&axi_wready;
 assign counters={ddr_data_beats,ddr_read_bursts,ddr_write_wait_cycles,
  ddr_read_gap_cycles,ddr_read_first_wait_cycles,ddr_cmd_wait_cycles,
  memory_active_cycles,engine_memory_cycles,engine_cmd_wait_cycles,
  idle_map_wait_cycles,idle_no_output_cycles,idle_no_source_cycles,
  repeated_frames,camera_skipped_frames,engine_busy_cycles,last_frame_cycles};
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  last_frame_cycles<=0;engine_busy_cycles<=0;
  idle_no_source_cycles<=0;idle_no_output_cycles<=0;idle_map_wait_cycles<=0;
  engine_cmd_wait_cycles<=0;engine_memory_cycles<=0;memory_active_cycles<=0;
  ddr_cmd_wait_cycles<=0;ddr_read_first_wait_cycles<=0;ddr_read_gap_cycles<=0;
  ddr_write_wait_cycles<=0;ddr_read_bursts<=0;ddr_data_beats<=0;
  memory_active<=0;memory_engine<=0;read_active<=0;read_seen_first<=0;write_active<=0;
 end else begin
  if(engine_done&&!engine_error) last_frame_cycles<=engine_cycles;
  if(engine_busy) engine_busy_cycles<=engine_busy_cycles+1'b1;
  // Mutually exclusive idle reasons, in the same prerequisites as launch.
  // A ready-to-launch cycle or start/finish bookkeeping is not an idle reason.
  if(!engine_busy&&!engine_inflight&&!engine_start) begin
   if(!map_available) idle_map_wait_cycles<=idle_map_wait_cycles+1'b1;
   else if(!source_ready) idle_no_source_cycles<=idle_no_source_cycles+1'b1;
   else if(!output_free) idle_no_output_cycles<=idle_no_output_cycles+1'b1;
  end
  if(engine_cmd_valid&&!engine_cmd_ready) engine_cmd_wait_cycles<=engine_cmd_wait_cycles+1'b1;
  // Client transaction ownership includes write preload, native service and
  // read drain. It is distinct from physical DDR data-bus occupancy.
  if(bridge_cmd_valid&&bridge_cmd_ready) begin memory_active<=1;memory_engine<=engine_cmd_ready;end
  if(memory_active) begin
   memory_active_cycles<=memory_active_cycles+1'b1;
   if(memory_engine) engine_memory_cycles<=engine_memory_cycles+1'b1;
   if(bridge_done) memory_active<=0;
  end
  if((axi_arvalid&&!axi_arready)||(axi_awvalid&&!axi_awready))
   ddr_cmd_wait_cycles<=ddr_cmd_wait_cycles+1'b1;
  if(read_start) begin
   ddr_read_bursts<=ddr_read_bursts+1'b1;
   read_active<=!(axi_rvalid&&axi_rlast);read_seen_first<=axi_rvalid;
  end else if(read_beat) begin
   read_seen_first<=1;
   if(axi_rlast) read_active<=0;
  end
  // Start counts as the first waiting cycle if no same-cycle data arrives.
  // A same-cycle AR/first R has zero first-data wait.
  if(read_pending&&!axi_rvalid) begin
   if(read_start||!read_seen_first) ddr_read_first_wait_cycles<=ddr_read_first_wait_cycles+1'b1;
   else ddr_read_gap_cycles<=ddr_read_gap_cycles+1'b1;
  end
  if(write_start) write_active<=!(axi_wready&&axi_wlast);
  else if(write_beat&&axi_wlast) write_active<=0;
  if(write_pending&&!axi_wready) ddr_write_wait_cycles<=ddr_write_wait_cycles+1'b1;
  if(read_beat&&write_beat) ddr_data_beats<=ddr_data_beats+2'd2;
  else if(read_beat||write_beat) ddr_data_beats<=ddr_data_beats+1'b1;
 end
endmodule
