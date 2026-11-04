// One UART owner. Existing V5 status bytes are unchanged; V6/LAT1 adds telemetry.
// Video never waits for UART: a bounded FIFO drops diagnostic samples when full.
module uart_status_latency_tx #(
 parameter CLK_HZ=100000000, BAUD=115200,
 parameter [31:0] BUILD_ID=32'h0720f001,
 parameter FIFO_DEPTH=8, POSE_EVENTS=0, VIEW_EVENTS=0
)(
 input wire clk,rst_n,input wire[15:0] config_version,flags,
 input wire[31:0] captured,processed,underflow,overflow,dropped,
 input wire[31:0] commits,rejected,map_cycles,raw_valid_frames,
 input wire[479:0] diagnostics,
 input wire[255:0] sccb_status,
 input wire[511:0] throughput_stats,
 input wire latency_valid,input wire[511:0] latency_sample,
 input wire[31:0] board_time_us,control_failures,map_invalid_pixels,map_domain_pixels,
 input wire[31:0] pose_guard_state,view_context,
 output reg[31:0] latency_dropped,
 output wire tx
);
 reg[31:0] timer,sample_ticks;
 reg status_pending,active,is_latency,is_pose,pose_pending;
 reg[31:0] seen_commits,seen_rejected,seen_failures;
 reg[15:0] seen_version;
 reg[4:0] seen_health;
 reg seen_building;
 reg[31:0] seen_guard,seen_context;
 wire pose_changed=POSE_EVENTS&&((commits!=seen_commits)||(rejected!=seen_rejected)||
       (control_failures!=seen_failures)||(config_version!=seen_version)||
       (seen_building&&!flags[1])||(seen_health!=flags[6:2])||
       (VIEW_EVENTS&&((seen_guard!=pose_guard_state)||(seen_context!=view_context))));
 reg[7:0] index;
 reg[1631:0] status_snapshot,snapshot;
 // Word15 is supplied by the sender, so only the lower 15 words are queued.
 reg[479:0] queue[0:FIFO_DEPTH-1];
 reg[479:0] enqueue_data;
 integer enqueue_address;
 reg enqueue_pending;
 integer write_ptr,read_ptr,count;
 reg[15:0] crc;
 reg[7:0] byte_data;
 wire ready;
 wire dequeue=!active&&!status_pending&&!pose_pending&&(count!=0);
 wire enqueue=latency_valid&&((count<FIFO_DEPTH)||dequeue);
 wire[7:0] protected_end=is_pose?(VIEW_EVENTS?8'd60:8'd56):(is_latency?8'd68:8'd208);
 wire[7:0] packet_last=is_pose?(VIEW_EVENTS?8'd61:8'd57):(is_latency?8'd69:8'd209);
 function[15:0] crc_byte(input[15:0] previous,input[7:0] value);
  reg[15:0] c;integer b;begin
   c=previous^{value,8'b0};
   for(b=0;b<8;b=b+1)c=c[15]?((c<<1)^16'h1021):(c<<1);
   crc_byte=c;
  end
 endfunction
 always @* begin
  if(index==0)byte_data=8'h55;
  else if(index==1)byte_data=8'haa;
  else if(index==2)byte_data=is_pose?(VIEW_EVENTS?8'h08:8'h07):(is_latency?8'h06:8'h05);
  else if(index==3)byte_data=is_pose?(VIEW_EVENTS?8'h38:8'h34):(is_latency?8'h40:8'hcc);
  else if(index==protected_end)byte_data=crc[15:8];
  else if(index==packet_last)byte_data=crc[7:0];
  else byte_data=snapshot[7:0];
 end
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) transmitter(
  .clk(clk),.rst_n(rst_n),.valid(active),.data(byte_data),.ready(ready),.tx(tx));
 // Preserve the original rising-edge admission/count decisions. Capture the
 // accepted payload/address, then commit RAM on the intervening falling edge.
 // A word accepted at edge N cannot be dequeued before edge N+1, so this adds
 // no visible latency and retains old-data semantics for a full FIFO exchange.
 // The half-cycle write path avoids the mailbox-to-LUTRAM same-edge hold path.
 always @(posedge clk) if(rst_n&&enqueue)begin
  enqueue_data<=latency_sample[479:0];enqueue_address<=write_ptr;
 end
 always @(negedge clk) if(rst_n&&enqueue_pending)
  queue[enqueue_address]<=enqueue_data;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  timer<=0;sample_ticks<=0;status_pending<=0;active<=0;is_latency<=0;is_pose<=0;
  pose_pending<=0;seen_commits<=0;seen_rejected<=0;seen_failures<=0;
  seen_version<=0;seen_building<=0;seen_health<=0;seen_guard<=0;seen_context<=0;
  index<=0;crc<=16'hffff;status_snapshot<=0;snapshot<=0;
  write_ptr<=0;read_ptr<=0;count<=0;latency_dropped<=0;enqueue_pending<=0;
 end else begin
  if(POSE_EVENTS) begin
   seen_commits<=commits;seen_rejected<=rejected;seen_failures<=control_failures;
   seen_version<=config_version;seen_building<=flags[1];
   seen_health<=flags[6:2];
   if(VIEW_EVENTS)begin seen_guard<=pose_guard_state;seen_context<=view_context;end
  end
  enqueue_pending<=enqueue;
  if(enqueue)begin
   if(write_ptr==FIFO_DEPTH-1)write_ptr<=0;else write_ptr<=write_ptr+1;
  end else if(latency_valid)latency_dropped<=latency_dropped+1'b1;
  if(dequeue)begin
   if(read_ptr==FIFO_DEPTH-1)read_ptr<=0;else read_ptr<=read_ptr+1;
  end
  case({enqueue,dequeue})
   2'b10:count<=count+1;
   2'b01:count<=count-1;
   default:count<=count;
  endcase
  if(!active)begin
   if(status_pending)begin
    snapshot<=status_snapshot;is_latency<=0;is_pose<=0;active<=1;
    index<=0;crc<=16'hffff;status_pending<=0;
   end else if(pose_pending&&(!VIEW_EVENTS||config_version==seen_version))begin
    // C127 publication metadata is registered by the configuration owner one
    // clock after the map-bank version changes. Let that owner catch up before
    // sampling a pending heartbeat/event, so a new version cannot carry the old
    // raw/mirror/scan state. C125/V7 and the periodic V5 path stay unchanged.
    // Coalesce event notifications, NOT transactions. Values and timestamp
    // describe the state sampled at this packet's start. Version/commit/failure
    // counters let the host distinguish publication from a rejected build.
    snapshot<={{1184{1'b0}},(VIEW_EVENTS?view_context:32'd0),pose_guard_state,map_domain_pixels,map_invalid_pixels,
      control_failures,processed,captured,board_time_us,map_cycles,rejected,
      commits,flags,config_version,BUILD_ID,32'h504f5331};
    is_pose<=1;is_latency<=0;active<=1;index<=0;crc<=16'hffff;pose_pending<=0;
   end else if(dequeue)begin
    // Timestamp/counter payload is little-endian. The sender owns word15.
    snapshot<={{1120{1'b0}},latency_dropped,queue[read_ptr][479:0]};
    is_latency<=1;is_pose<=0;active<=1;index<=0;crc<=16'hffff;
   end
  end
  // Event-only POS1 leaves a host connected after initialization unable to
  // discover C125 capabilities until another configuration event occurs.
  // Reuse the one-second timer for a half-period-offset heartbeat. Coalesce
  // it with events in the existing bounded pending bit; the packet still
  // snapshots current state/time at start and preserves packet priorities.
  // Set wins over simultaneous dequeue, including an event racing heartbeat
  // start. UART stalls never stall video or configuration publication.
  if(pose_changed||(POSE_EVENTS&&(timer==CLK_HZ/2)))pose_pending<=1;
  // Keep V5 board-second cadence and its exact 51-word layout. A pending V5
  // starts at the next packet boundary and cannot be starved by V6 traffic.
  if(timer==CLK_HZ-1)begin
   timer<=0;sample_ticks<=sample_ticks+1'b1;status_pending<=1;
   status_snapshot<={throughput_stats,sccb_status,diagnostics,
    sample_ticks+32'd1,raw_valid_frames,map_cycles,rejected,commits,dropped,
    overflow,underflow,processed,captured,flags,config_version,BUILD_ID};
  end else timer<=timer+1'b1;
  if(active&&ready)begin
   if(index<protected_end)crc<=crc_byte(crc,byte_data);
   if(index>=4&&index<protected_end)snapshot<=snapshot>>8;
   if(index==packet_last)active<=0;else index<=index+1'b1;
  end
 end
endmodule
