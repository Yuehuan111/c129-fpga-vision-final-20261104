// Diagnostic time only. 32-bit microseconds wrap after about 71.6 minutes.
module latency_timebase #(parameter CLOCK_HZ=100000000)(
 input wire clk,rst_n,output reg[31:0] time_us,
 (* syn_preserve=1 *) output reg[31:0] time_gray
);
 localparam DIVISOR=CLOCK_HZ/1000000;
 reg[15:0] divider;
 wire[31:0] next_time=time_us+1'b1;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin divider<=0;time_us<=0;time_gray<=0;end
 else if(divider==DIVISOR-1) begin
  divider<=0;time_us<=next_time;time_gray<=next_time^(next_time>>1);
 end else divider<=divider+1'b1;
endmodule

// The source Gray code is registered. Physical skew constraints must cover
// source_gray -> gray_sync1; no combinational binary bus is sampled across CDC.
module latency_gray_receiver(
 input wire clk,rst_n,input wire[31:0] source_gray,output reg[31:0] time_us
);
 (* syn_preserve=1,syn_srlstyle="registers" *) reg[31:0] gray_sync1,gray_sync2;
 wire[31:0] decoded;
 genvar k;
 generate for(k=0;k<32;k=k+1) begin: decode
  assign decoded[k]=^gray_sync2[31:k];
 end endgenerate
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin gray_sync1<=0;gray_sync2<=0;time_us<=0;end
 else begin gray_sync1<=source_gray;gray_sync2<=gray_sync1;time_us<=decoded;end
endmodule
