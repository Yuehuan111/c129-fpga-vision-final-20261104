// One-entry, acknowledged bundled-data CDC. Payload stays immutable from
// source acceptance until destination capture is acknowledged. An extra
// destination settling cycle follows the two-flop request synchronizer.
// Both ends must be reset together (domain-local synchronous release is OK).
module latency_mailbox #(parameter WIDTH=32)(
 input wire source_clk,source_rst_n,source_valid,input wire[WIDTH-1:0] source_data,
 output wire source_busy,output reg source_overrun,
 input wire destination_clk,destination_rst_n,
 output reg destination_valid,output reg[WIDTH-1:0] destination_data
);
 (* syn_preserve=1 *) reg[WIDTH-1:0] held_data;
 (* syn_preserve=1 *) reg request_toggle,ack_toggle;
 (* syn_preserve=1,syn_srlstyle="registers" *) reg ack_sync1,ack_sync2;
 (* syn_preserve=1,syn_srlstyle="registers" *) reg request_sync1,request_sync2;
 reg settling;
 assign source_busy=request_toggle!=ack_sync2;
 always @(posedge source_clk or negedge source_rst_n)
 if(!source_rst_n) begin held_data<=0;request_toggle<=0;ack_sync1<=0;ack_sync2<=0;source_overrun<=0;end
 else begin
  ack_sync1<=ack_toggle;ack_sync2<=ack_sync1;source_overrun<=0;
  if(source_valid) begin
   if(!source_busy) begin held_data<=source_data;request_toggle<=~request_toggle;end
   else source_overrun<=1;
  end
 end
 always @(posedge destination_clk or negedge destination_rst_n)
 if(!destination_rst_n) begin
  request_sync1<=0;request_sync2<=0;ack_toggle<=0;settling<=0;
  destination_valid<=0;destination_data<=0;
 end else begin
  request_sync1<=request_toggle;request_sync2<=request_sync1;destination_valid<=0;
  if(!settling) begin if(request_sync2!=ack_toggle) settling<=1;end
  else begin
   destination_data<=held_data;destination_valid<=1;
   ack_toggle<=request_sync2;settling<=0;
  end
 end
endmodule
