// One outstanding aligned 256-bit transaction for the vendor Logos DDR3_50H simplified AXI.
// req_addr is a BYTE address. HMIC_S address counts 32-bit DRAM words: addr >> 2.
// No AXI WVALID/B/RREADY exist at the vendor IP. Hold data BEFORE issuing AW.
module ddr_single_beat(
 input wire clk,rst_n,
 input wire req_valid,req_write,input wire[31:0] req_addr,
 input wire[255:0] req_wdata,input wire[31:0] req_wstrb,output wire req_ready,
 output reg rsp_valid,output reg[255:0] rsp_data,input wire rsp_ready,
 output wire[27:0] axi_awaddr,output wire[3:0] axi_awlen,
 output wire axi_awvalid,input wire axi_awready,
 output wire[255:0] axi_wdata,output wire[31:0] axi_wstrb,
 input wire axi_wready,axi_wlast,
 output wire[27:0] axi_araddr,output wire[3:0] axi_arlen,
 output wire axi_arvalid,input wire axi_arready,
 input wire[255:0] axi_rdata,input wire axi_rvalid,axi_rlast
);
 localparam IDLE=0,AW=1,WD=2,AR=3,RD=4,RESP=5;
 reg[2:0] state;reg[31:0] addr,strb;reg[255:0] wd;
 assign req_ready=rst_n && state==IDLE;
 assign axi_awaddr=addr[29:2];assign axi_araddr=addr[29:2];
 assign axi_awlen=0;assign axi_arlen=0;
 assign axi_awvalid=rst_n && state==AW;
 assign axi_arvalid=rst_n && state==AR;
 assign axi_wdata=wd;assign axi_wstrb=strb;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin state<=IDLE;addr<=0;strb<=0;wd<=0;rsp_valid<=0;rsp_data<=0;end
 else case(state)
  IDLE: if(req_valid) begin
   addr<=req_addr;wd<=req_wdata;strb<=req_wstrb;state<=req_write?AW:AR;
  end
  AW: if(axi_awready) begin
   if(axi_wready&&axi_wlast) begin rsp_valid<=1;rsp_data<=0;state<=RESP;end
   else state<=WD;
  end
  WD: if(axi_wready&&axi_wlast) begin rsp_valid<=1;rsp_data<=0;state<=RESP;end
  AR: if(axi_arready) begin
   if(axi_rvalid&&axi_rlast) begin rsp_valid<=1;rsp_data<=axi_rdata;state<=RESP;end
   else state<=RD;
  end
  RD: if(axi_rvalid&&axi_rlast) begin rsp_valid<=1;rsp_data<=axi_rdata;state<=RESP;end
  RESP: if(rsp_ready) begin rsp_valid<=0;state<=IDLE;end
  default: state<=IDLE;
 endcase
endmodule
