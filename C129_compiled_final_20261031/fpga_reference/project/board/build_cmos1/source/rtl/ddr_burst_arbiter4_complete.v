// Four clients; an accepted command owns data and completion until b_done.
// 0=deadline display, 1=capture, 2/3=map/warp round-robin.
// A warp no longer excludes map writes for its entire frame. Finite bursts
// share DDR fairly; display and capture retain strict deadline priority.
// No real-time guarantee is inferred under unbounded external DDR stalls.
// A previously presented/stalled command remains locked until accepted.
module ddr_burst_arbiter4_complete(
 input wire clk,rst_n,warp_inflight,
 input wire[3:0] cmd_valid,cmd_write,output wire[3:0] cmd_ready,
 input wire[127:0] cmd_addr,input wire[15:0] cmd_len,
 input wire[3:0] w_valid,output wire[3:0] w_ready,
 input wire[1023:0] w_data,input wire[127:0] w_strb,
 output wire[3:0] r_valid,input wire[3:0] r_ready,
 output wire[1023:0] r_data,output wire[3:0] r_last,done,error,
 output wire b_cmd_valid,b_cmd_write,input wire b_cmd_ready,
 output wire[31:0] b_cmd_addr,output wire[3:0] b_cmd_len,
 output wire b_w_valid,input wire b_w_ready,
 output wire[255:0] b_w_data,output wire[31:0] b_w_strb,
 input wire b_r_valid,output wire b_r_ready,input wire[255:0] b_r_data,
 input wire b_r_last,b_done,b_error
);
 reg active,rr,pending;reg[1:0] owner,pending_owner;
 wire[1:0] choice=cmd_valid[0]?2'd0:cmd_valid[1]?2'd1:
                    ((cmd_valid[2]&&cmd_valid[3])?(rr?2'd3:2'd2):cmd_valid[2]?2'd2:2'd3);
 wire[1:0] grant=pending?pending_owner:choice;
 wire eligible=|cmd_valid;
 assign b_cmd_valid=rst_n && !active && (pending||eligible);
 assign b_cmd_write=cmd_write[grant];
 assign b_cmd_addr=cmd_addr[grant*32+:32];assign b_cmd_len=cmd_len[grant*4+:4];
 assign b_w_valid=active && w_valid[owner];
 assign b_w_data=w_data[owner*256+:256];assign b_w_strb=w_strb[owner*32+:32];
 assign b_r_ready=active && r_ready[owner];
 genvar k;
 generate for(k=0;k<4;k=k+1) begin: route
  assign cmd_ready[k]=b_cmd_valid && b_cmd_ready && grant==k;
  assign w_ready[k]=active && owner==k && b_w_ready;
  assign r_valid[k]=active && owner==k && b_r_valid;
  assign r_data[k*256+:256]=b_r_data;
  assign r_last[k]=active && owner==k && b_r_last;
  assign done[k]=active && owner==k && b_done;
  assign error[k]=active && owner==k && b_error;
 end endgenerate
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin active<=0;owner<=0;rr<=0;pending<=0;pending_owner<=0;end
 else begin
  if(b_cmd_valid && !b_cmd_ready && !pending) begin pending<=1;pending_owner<=grant;end
  if(b_cmd_valid && b_cmd_ready) begin active<=1;owner<=grant;pending<=0;end
  if(active && b_done) begin
   active<=0;
   if(owner==2) rr<=1;
   if(owner==3) rr<=0;
  end
 end
endmodule
