// Dual-clock FIFO with Gray pointers and a registered read-side holding word.
// DEPTH=2**ADDR_BITS. Reset must reach BOTH domains before traffic resumes.
module async_pixel_fifo #(parameter WIDTH=18, ADDR_BITS=10)(
 input wire wclk, wrst_n, input wire wvalid, input wire [WIDTH-1:0] wdata,
 output wire wready, input wire rclk, rrst_n, output reg rvalid,
 output reg [WIDTH-1:0] rdata, input wire rready
);
 localparam PTR=ADDR_BITS+1;
 reg [WIDTH-1:0] mem[0:(1<<ADDR_BITS)-1];
 reg [PTR-1:0] wb,rb;
 // Preserve Gray MSBs rather than merging them into the binary pointers: the
 // generated PDS FDC constrains these exact Gray source -> first-stage paths.
 (* syn_preserve=1 *) reg [PTR-1:0] wg,rg;
 (* syn_preserve=1, syn_srlstyle="registers" *) reg [PTR-1:0] rg1,rg2,wg1,wg2;
 wire [PTR-1:0] wb_next=wb+1'b1;
 wire [PTR-1:0] wg_next=(wb_next>>1)^wb_next;
 // Reserve one slot. Comparing next pointer simplifies the full combinational path.
 wire full=(wg_next=={~rg2[PTR-1:PTR-2],rg2[PTR-3:0]});
 wire empty=(rg==wg2);
 wire fetch=!empty && (!rvalid || rready);
 assign wready=wrst_n && !full;
 always @(posedge wclk) if(wvalid && wready) mem[wb[ADDR_BITS-1:0]]<=wdata;
 always @(posedge rclk) if(fetch && rrst_n) rdata<=mem[rb[ADDR_BITS-1:0]];
 always @(posedge wclk or negedge wrst_n)
 if(!wrst_n) begin wb<=0;wg<=0;rg1<=0;rg2<=0;end
 else begin
  rg1<=rg;rg2<=rg1;
  if(wvalid&&wready) begin wb<=wb_next;wg<=wg_next;end
 end
 always @(posedge rclk or negedge rrst_n)
 if(!rrst_n) begin rb<=0;rg<=0;wg1<=0;wg2<=0;rvalid<=0;end
 else begin
  wg1<=wg;wg2<=wg1;
  if(fetch) begin rb<=rb+1'b1;rg<=((rb+1'b1)>>1)^(rb+1'b1);rvalid<=1;end
  else if(rready) rvalid<=0;
 end
endmodule
