// Hold the video PLL in reset for 20ms after FPGA configuration at50MHz.
// Independent of PLL outputs, DDR initialization and the video algorithm.
module pll_startup_reset #(parameter integer HOLD_CYCLES=1000000)(
 input wire clk,output wire reset
);
 localparam integer WIDTH=$clog2(HOLD_CYCLES+1);
 reg [WIDTH-1:0] count=0;
 assign reset=(count<HOLD_CYCLES);
 always @(posedge clk)
  if(count<HOLD_CYCLES) count<=count+1'b1;
endmodule
