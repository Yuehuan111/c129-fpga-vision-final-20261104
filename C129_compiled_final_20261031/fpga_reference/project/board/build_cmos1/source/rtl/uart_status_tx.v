// Once per board second, snapshot 51 little-endian 32-bit words.
// Packet: 55 AA 05 CC, 204 payload bytes, CRC16/CCITT-FALSE big endian.
// Payload word0 build id, word1 {flags[15:0],config_version[15:0]}, then counters.
// Words 0..9 preserve v1. Word10 raw validated frames, word11 board second tick.
module uart_status_tx #(parameter CLK_HZ=100000000,BAUD=115200,
 parameter[31:0] BUILD_ID=32'h10800001)(
 input wire clk,rst_n,input wire[15:0] config_version,flags,
 input wire[31:0] captured,processed,underflow,overflow,dropped,
 input wire[31:0] commits,rejected,map_cycles,raw_valid_frames,
 input wire[479:0] diagnostics,
 input wire[255:0] sccb_status,
 input wire[511:0] throughput_stats,
 output wire tx
);
 reg[31:0] timer,sample_ticks;reg active;reg[7:0] index;
 reg[1631:0] snapshot;reg[15:0] crc;reg[7:0] byte_data;
 wire ready;
 function[15:0] crc_byte(input[15:0] previous,input[7:0] value);
  reg[15:0] c;integer b;begin
   c=previous^{value,8'b0};
   for(b=0;b<8;b=b+1) c=c[15]?((c<<1)^16'h1021):(c<<1);
   crc_byte=c;
  end
 endfunction
 always @* begin
  case(index)
   0:byte_data=8'h55;1:byte_data=8'haa;2:byte_data=8'h05;3:byte_data=8'hcc;
   208:byte_data=crc[15:8];209:byte_data=crc[7:0];
   default:byte_data=snapshot[7:0];
  endcase
 end
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) transmitter(
  .clk(clk),.rst_n(rst_n),.valid(active),.data(byte_data),.ready(ready),.tx(tx));
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin timer<=0;sample_ticks<=0;active<=0;index<=0;crc<=16'hffff;snapshot<=0;end
 else begin
  if(timer==CLK_HZ-1) begin
   timer<=0;sample_ticks<=sample_ticks+1'b1;
   if(!active) begin
    snapshot<={throughput_stats,sccb_status,diagnostics,sample_ticks+32'd1,raw_valid_frames,map_cycles,rejected,commits,dropped,overflow,underflow,processed,captured,
               flags,config_version,BUILD_ID};
    index<=0;crc<=16'hffff;active<=1;
   end
  end else timer<=timer+1'b1;
  if(active&&ready) begin
   if(index<208) crc<=crc_byte(crc,byte_data);
   if(index>=4 && index<208) snapshot<=snapshot>>8;
   if(index==209) active<=0;else index<=index+1'b1;
  end
 end
endmodule
