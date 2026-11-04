// Logos DDR3_50H: 256-bit beats, address unit is a 32-bit DRAM word.
// There is no WVALID, B channel or RREADY. Buffer a complete write before AW;
// reserve a complete read burst before AR, so an unstoppable return never drops.
// One command is outstanding. cmd_len is beats-1 (0..15); addresses are bytes.
// Split only physical commands at 4 KiB boundaries. Client beat indices and
// completion span both segments; no vendor-specific cross-page behavior is used.
module ddr_burst_bridge(
 input wire clk,rst_n,
 input wire cmd_valid,cmd_write,input wire[31:0] cmd_addr,input wire[3:0] cmd_len,
 output wire cmd_ready,
 input wire w_valid,input wire[255:0] w_data,input wire[31:0] w_strb,output wire w_ready,
 output wire r_valid,output wire[255:0] r_data,output wire r_last,input wire r_ready,
 output reg done,error,
 output wire[27:0] axi_awaddr,output wire[3:0] axi_awlen,
 output wire axi_awvalid,input wire axi_awready,
 output wire[255:0] axi_wdata,output wire[31:0] axi_wstrb,
 input wire axi_wready,axi_wlast,
 output wire[27:0] axi_araddr,output wire[3:0] axi_arlen,
 output wire axi_arvalid,input wire axi_arready,
 input wire[255:0] axi_rdata,input wire axi_rvalid,axi_rlast
);
 localparam IDLE=0,LOAD=1,AW=2,WRITE=3,AR=4,READ=5,DRAIN=6;
 reg[2:0] state;
 reg[31:0] addr;
 reg[3:0] length,load_index,write_index,read_last_index;
 reg[3:0] segment_length,segment_last;
 reg[4:0] received,consumed;
 reg[255:0] payload[0:15];
 reg[31:0] masks[0:15];
 wire accept_read=(state==READ || (state==AR && axi_arready)) && axi_rvalid;
 wire accept_write=(state==WRITE || (state==AW && axi_awready)) && axi_wready;
 // At most 16 beats: only the final 15 beat positions of a page can cross.
 wire[3:0] page_last=~cmd_addr[8:5];
 wire[3:0] first_length=((&cmd_addr[11:9]) && cmd_len>page_last)?page_last:cmd_len;
 assign cmd_ready=rst_n && state==IDLE;
 assign w_ready=rst_n && state==LOAD;
 assign r_valid=rst_n && (state==AR || state==READ || state==DRAIN) && consumed<received;
 assign r_data=payload[consumed[3:0]];
 assign r_last=(state==DRAIN) && consumed[3:0]==read_last_index;
 assign axi_awaddr=addr[29:2];assign axi_araddr=addr[29:2];
 assign axi_awlen=segment_length;assign axi_arlen=segment_length;
 assign axi_awvalid=rst_n && state==AW;
 assign axi_arvalid=rst_n && state==AR;
 assign axi_wdata=payload[write_index];assign axi_wstrb=masks[write_index];
 // No memory reset: payload is visible only after a valid producer write.
 always @(posedge clk) begin
  if(w_ready && w_valid) begin payload[load_index]<=w_data;masks[load_index]<=w_strb;end
  else if(rst_n && accept_read) payload[received[3:0]]<=axi_rdata;
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=IDLE;addr<=0;length<=0;load_index<=0;write_index<=0;
  segment_length<=0;segment_last<=0;
  received<=0;consumed<=0;read_last_index<=0;done<=0;error<=0;
 end else begin
  done<=0;
  if(r_valid && r_ready) begin
   consumed<=consumed+1'b1;
   if(r_last) begin state<=IDLE;done<=1;end
  end
  case(state)
   IDLE: if(cmd_valid) begin
    addr<=cmd_addr;length<=cmd_len;load_index<=0;write_index<=0;
    segment_length<=first_length;segment_last<=first_length;
    received<=0;consumed<=0;read_last_index<=cmd_len;error<=0;
    if(cmd_addr[4:0]!=0 || cmd_addr[31:30]!=0) begin done<=1;error<=1;end
    else state<=cmd_write?LOAD:AR;
   end
   LOAD: if(w_valid) begin
    if(load_index==length) state<=AW;
    else load_index<=load_index+1'b1;
   end
   AW: if(axi_awready) state<=WRITE;
   AR: if(axi_arready) state<=READ;
   default: ;
  endcase
  // First data may coincide with command acceptance; do not lose that beat.
  if(accept_write) begin
   if(axi_wlast!=(write_index==segment_last)) error<=1;
   if(axi_wlast || write_index==segment_last) begin
    if(axi_wlast && write_index==segment_last && segment_last!=length) begin
     addr<=addr+(({28'b0,segment_length}+32'd1)<<5);
     segment_length<=length-segment_last-1'b1;segment_last<=length;
     write_index<=write_index+1'b1;state<=AW;
    end else begin state<=IDLE;done<=1;end
   end
   else write_index<=write_index+1'b1;
  end
  if(accept_read) begin
   received<=received+1'b1;
   if(axi_rlast!=(received=={1'b0,segment_last})) error<=1;
   if(axi_rlast || received=={1'b0,segment_last}) begin
    if(axi_rlast && received=={1'b0,segment_last} && segment_last!=length) begin
     addr<=addr+(({28'b0,segment_length}+32'd1)<<5);
     segment_length<=length-segment_last-1'b1;segment_last<=length;state<=AR;
    end else begin read_last_index<=received[3:0];state<=DRAIN;end
   end
  end
 end
endmodule
