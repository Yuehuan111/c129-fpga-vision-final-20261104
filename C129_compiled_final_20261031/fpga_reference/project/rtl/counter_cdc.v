// Statistics only: sample a monotonically incrementing binary counter without
// torn multi-bit values. Source Gray encoding MUST be registered, not combinational.
// Each source update must be 0 or +1 (apart from reset); destination may skip values.
module counter_cdc #(parameter WIDTH=32)(
 input wire source_clk,source_rst_n,input wire[WIDTH-1:0] source_binary,
 input wire destination_clk,destination_rst_n,output reg[WIDTH-1:0] destination_binary
);
 // PDS ADS supports syn_preserve/syn_srlstyle; ASYNC_REG is ignored by PDS.
 // Physical Gray-bus skew is constrained separately in the generated FDC.
 (* syn_preserve=1 *) reg[WIDTH-1:0] source_gray;
 (* syn_preserve=1, syn_srlstyle="registers" *) reg[WIDTH-1:0] gray_sync1,gray_sync2;
 wire[WIDTH-1:0] decoded;
 genvar k;
 generate for(k=0;k<WIDTH;k=k+1) begin: decode
  assign decoded[k]=^gray_sync2[WIDTH-1:k];
 end endgenerate
 always @(posedge source_clk or negedge source_rst_n)
 if(!source_rst_n) source_gray<=0;
 else source_gray<=source_binary^(source_binary>>1);
 always @(posedge destination_clk or negedge destination_rst_n)
 if(!destination_rst_n) begin gray_sync1<=0;gray_sync2<=0;destination_binary<=0;end
 else begin gray_sync1<=source_gray;gray_sync2<=gray_sync1;destination_binary<=decoded;end
endmodule
