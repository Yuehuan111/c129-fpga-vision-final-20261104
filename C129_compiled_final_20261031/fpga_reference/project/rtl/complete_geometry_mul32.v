// Three-stage signed 32x32 multiplier, global clock enable supports backpressure.
// Four registered 16-bit products keep the 100 MHz path within one DSP multiply
// followed by registered adders. No unregistered 32-bit multiplier cascade.
module complete_geometry_mul32(input wire clk,ce,
 input wire signed [31:0] a,b,output reg signed [63:0] product);
 reg [31:0] p00,l2;
 reg signed [32:0] p01,p10;
 reg signed [31:0] p11;
 reg signed [33:0] cross2;
 reg signed [31:0] h2;
 always @(posedge clk) if(ce) begin
  p00<=$unsigned(a[15:0])*$unsigned(b[15:0]);
  p01<=$signed(a[31:16])*$signed({1'b0,b[15:0]});
  p10<=$signed({1'b0,a[15:0]})*$signed(b[31:16]);
  p11<=$signed(a[31:16])*$signed(b[31:16]);
  l2<=p00;h2<=p11;cross2<={{1{p01[32]}},p01}+{{1{p10[32]}},p10};
  product<=($signed({{32{h2[31]}},h2})<<<32)+
    ($signed({{30{cross2[33]}},cross2})<<<16)+$signed({32'b0,l2});
 end
endmodule
