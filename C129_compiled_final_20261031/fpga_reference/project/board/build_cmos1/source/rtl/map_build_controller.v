// FPGA-generated lookup table. Parameters are applied atomically.
// Map: 64 bits per output pixel, low 39 bits retain brown_mapper's contract.
// Pause new processing while switching/rebuilding; the display stays live.
// Each successful bank retains its 14-word geometry key. Gain/offset do not
// affect coordinates, so two cached geometries can be reused after draining.
module map_build_controller #(
 parameter IMG_WIDTH=1920, IMG_HEIGHT=1080, FISHEYE_MODEL=0,
 parameter [31:0] MAP0_BASE=32'h01800000, MAP1_BASE=32'h02800000
)(
 input wire clk,rst_n,
 input wire cfg_valid,input wire[4:0] cfg_addr,input wire[31:0] cfg_data,
 input wire cfg_commit,output wire cfg_ready,cfg_commit_ready,
 input wire engine_busy,engine_start,
 output reg map_available,output wire map_building,output reg[31:0] map_base,
 output reg[15:0] config_version,
 output reg signed[31:0] gain_q8,offset,
 output reg error_sticky,output reg[31:0] rebuild_cycles,
 output wire cmd_valid,input wire cmd_ready,output wire cmd_write,
 output wire[31:0] cmd_addr,output wire[3:0] cmd_len,
 output wire w_valid,input wire w_ready,output wire[255:0] w_data,
 output wire[31:0] w_strb,input wire mem_done,mem_error
);
 localparam IDLE=0,DRAIN=1,CHECK=2,START=3,COLLECT=4,COMMAND=5,WRITE=6,WAIT_DONE=7;
 reg[2:0] state;
 reg[1:0] quiet;
 localparam REGISTER_COUNT=FISHEYE_MODEL?18:16,GEOMETRY_COUNT=FISHEYE_MODEL?16:14;
 reg[31:0] shadow[0:17];reg[17:0] written;
 reg bank,build_bank,last_batch,build_failed;
 reg[31:0] bank0_geometry[0:15],bank1_geometry[0:15];
 reg[1:0] cache_valid;
 reg[4:0] compare_index;
 reg match0,match1;
 wire[4:0] compare_address=FISHEYE_MODEL&&compare_index>=14?compare_index+5'd2:compare_index;
 wire word_match0=shadow[compare_address]==bank0_geometry[compare_index];
 wire word_match1=shadow[compare_address]==bank1_geometry[compare_index];
 // A failed DDR batch must discard the mapper's partially generated frame.
 // Registered reset avoids a combinational state-decode on its async reset.
 reg mapper_reset_n;
 reg[255:0] pack_data;
 reg[1:0] lane;
 reg[4:0] fill_count,send_index,batch_beats;
 (* syn_ramstyle="block_ram" *) reg[255:0] batch[0:15];
 reg[255:0] batch_q;
 reg[31:0] write_offset;
 wire mapper_valid,mapper_last,mapper_busy,mapper_done,mapper_error;
 wire[38:0] mapper_entry;
 integer i;
 assign cfg_ready=rst_n && state==IDLE && !cfg_commit;
 // Explicit model tag and mode protect the new18-word contract from old UART senders.
 assign cfg_commit_ready=rst_n && state==IDLE && (&written[REGISTER_COUNT-1:0]) &&
  (!FISHEYE_MODEL||(shadow[17]==32'h46495348&&(shadow[16]==0||shadow[16]==1)));
 assign map_building=state!=IDLE;
 assign cmd_valid=state==COMMAND;
 assign cmd_write=1'b1;
 assign cmd_addr=(build_bank?MAP1_BASE:MAP0_BASE)+write_offset;
 assign cmd_len=batch_beats-1'b1;
 assign w_valid=state==WRITE;
 assign w_data=batch_q;
 assign w_strb=32'hffffffff;
 generate if(FISHEYE_MODEL) begin: fish_geometry
 fisheye_mapper #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) mapper(
  .clk(clk),.rst_n(rst_n && mapper_reset_n),.start(state==START),
  .cfg_fx_src(shadow[0]),.cfg_fy_src(shadow[1]),.cfg_cx_src(shadow[2]),.cfg_cy_src(shadow[3]),
  .cfg_cx_out(shadow[4]),.cfg_cy_out(shadow[5]),.cfg_fx_out(shadow[6]),.cfg_fy_out(shadow[7]),
  .cfg_k1(shadow[8]),.cfg_k2(shadow[9]),.cfg_k3(shadow[10]),.cfg_k4(shadow[11]),
  .cfg_cos(shadow[12]),.cfg_sin(shadow[13]),.cfg_mode(shadow[16]),.map_ready(state==COLLECT),
  .busy(mapper_busy),.done(mapper_done),.map_valid(mapper_valid),.map_entry(mapper_entry),
  .map_last(mapper_last),.error(mapper_error));
 end else begin: brown_geometry
 brown_mapper #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) mapper(
  .clk(clk),.rst_n(rst_n && mapper_reset_n),.start(state==START),
  .cfg_fx_src(shadow[0]),.cfg_fy_src(shadow[1]),.cfg_cx_src(shadow[2]),.cfg_cy_src(shadow[3]),
  .cfg_cx_out(shadow[4]),.cfg_cy_out(shadow[5]),.cfg_inv_fx_out(shadow[6]),.cfg_inv_fy_out(shadow[7]),
  .cfg_k1(shadow[8]),.cfg_k2(shadow[9]),.cfg_p1(shadow[10]),.cfg_p2(shadow[11]),
  .cfg_cos(shadow[12]),.cfg_sin(shadow[13]),.map_ready(state==COLLECT),
  .busy(mapper_busy),.done(mapper_done),.map_valid(mapper_valid),.map_entry(mapper_entry),
  .map_last(mapper_last),.error(mapper_error));
 end endgenerate
 // Independent synchronous RAM ports are required for DRM inference. The
 // command cycle primes beat 0; each accepted beat prefetches its successor.
 // During backpressure batch_q is stable. Memory has no async reset, and its
 // contents are only consumed after all entries in this batch were written.
 always @(posedge clk) begin
  if(state==COLLECT && mapper_valid && (lane==3 || mapper_last))
   batch[fill_count[3:0]]<={{25'b0,mapper_entry},pack_data[191:0]};
  if(state==COMMAND) batch_q<=batch[0];
  else if(state==WRITE && w_ready) batch_q<=batch[send_index[3:0]+4'd1];
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;written<=0;quiet<=0;bank<=1;build_bank<=0;map_base<=MAP0_BASE;
   map_available<=0;config_version<=0;gain_q8<=256;offset<=0;error_sticky<=0;
   pack_data<=0;lane<=0;fill_count<=0;send_index<=0;batch_beats<=0;write_offset<=0;
   last_batch<=0;build_failed<=0;rebuild_cycles<=0;mapper_reset_n<=0;
   cache_valid<=0;compare_index<=0;match0<=0;match1<=0;
   for(i=0;i<18;i=i+1) shadow[i]<=0;
  end else begin
   if(map_building) rebuild_cycles<=rebuild_cycles+1'b1;
   if(state>=COLLECT && mapper_error) begin build_failed<=1;error_sticky<=1;end
   if(cfg_valid && cfg_ready && cfg_addr<REGISTER_COUNT) begin
    shadow[cfg_addr]<=cfg_data;written[cfg_addr]<=1;
   end
   case(state)
    IDLE: begin
     mapper_reset_n<=0;
     if(cfg_commit && cfg_commit_ready) begin
     written<=0;map_available<=0;quiet<=0;state<=DRAIN;
     build_bank<=~bank;pack_data<=0;lane<=0;fill_count<=0;send_index<=0;
     write_offset<=0;last_batch<=0;build_failed<=0;rebuild_cycles<=0;
     end
    end
    DRAIN: if(engine_busy || engine_start) quiet<=0;
     else if(quiet==2) begin
      compare_index<=0;match0<=cache_valid[0];match1<=cache_valid[1];state<=CHECK;
     end
     else quiet<=quiet+1'b1;
    CHECK: begin
     // Compare one word per cycle, avoiding a 448-bit reduction in the
     // 100 MHz publication path. Shadow is locked until returning to IDLE.
     if(engine_busy || engine_start) begin quiet<=0;state<=DRAIN;end
     else if(compare_index==GEOMETRY_COUNT-1) begin
      if((match0 && word_match0) || (match1 && word_match1)) begin
       bank<=!(match0 && word_match0);
       map_base<=(match0 && word_match0)?MAP0_BASE:MAP1_BASE;
       gain_q8<=shadow[14];offset<=shadow[15];config_version<=config_version+1'b1;
       map_available<=1;mapper_reset_n<=0;state<=IDLE;
      end else begin
       // A partially overwritten bank must never match a later request,
       // including after memory or arithmetic failure. The other bank is
       // still reusable, but unavailable while this transaction is active.
       cache_valid[build_bank]<=0;mapper_reset_n<=1;state<=START;
      end
     end else begin
      match0<=match0 && word_match0;match1<=match1 && word_match1;
      compare_index<=compare_index+1'b1;
     end
    end
    START: state<=COLLECT;
    COLLECT: if(mapper_valid) begin
     pack_data[lane*64+:64]<={25'b0,mapper_entry};
     if(lane==3 || mapper_last) begin
      // Dimensions must contain a multiple of four pixels. All supported
      // video/tile modes satisfy this, so every written beat is complete.
      pack_data<=0;lane<=0;
      if(fill_count==15 || mapper_last) begin
       batch_beats<=fill_count+1'b1;fill_count<=0;send_index<=0;
       last_batch<=mapper_last;state<=COMMAND;
      end else fill_count<=fill_count+1'b1;
     end else lane<=lane+1'b1;
    end
    COMMAND: if(cmd_ready) state<=WRITE;
    WRITE: if(w_ready) begin
     if(send_index+1==batch_beats) state<=WAIT_DONE;
     else send_index<=send_index+1'b1;
    end
    WAIT_DONE: if(mem_done) begin
     if(mem_error) begin
      error_sticky<=1;map_available<=0;mapper_reset_n<=0;state<=IDLE;
     end
     else if(last_batch) begin
      if(!build_failed && !mapper_error) begin
       bank<=build_bank;map_base<=build_bank?MAP1_BASE:MAP0_BASE;
       gain_q8<=shadow[14];offset<=shadow[15];config_version<=config_version+1'b1;
       map_available<=1;
       cache_valid[build_bank]<=1;
       for(i=0;i<GEOMETRY_COUNT;i=i+1) begin
        if(build_bank) bank1_geometry[i]<=shadow[i<14?i:i+2];
        else bank0_geometry[i]<=shadow[i<14?i:i+2];
       end
      end else begin error_sticky<=1;map_available<=0;end
      mapper_reset_n<=0;state<=IDLE;
     end else begin write_offset<=write_offset+{22'b0,batch_beats,5'b0};state<=COLLECT;end
    end
    default: begin mapper_reset_n<=0;state<=IDLE;end
   endcase
  end
 end
endmodule

