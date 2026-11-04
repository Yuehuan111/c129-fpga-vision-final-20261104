// 8N1 transmitter; valid/ready accepts a byte only while idle.
module uart_tx #(parameter CLK_HZ=100000000,BAUD=115200)(
 input wire clk,rst_n,input wire valid,input wire[7:0] data,
 output wire ready,output wire tx
);
 localparam DIV=(CLK_HZ+BAUD/2)/BAUD;
 reg[31:0] count;reg[3:0] bit_index;reg[9:0] shift;reg busy;
 assign ready=rst_n&&!busy;assign tx=busy?shift[0]:1'b1;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin count<=0;bit_index<=0;shift<=10'h3ff;busy<=0;end
 else if(!busy) begin
  if(valid) begin shift<={1'b1,data,1'b0};count<=DIV-1;bit_index<=0;busy<=1;end
 end else if(count!=0) count<=count-1'b1;
 else if(bit_index==9) begin busy<=0;shift<=10'h3ff;end
 else begin shift<={1'b1,shift[9:1]};bit_index<=bit_index+1'b1;count<=DIV-1;end
endmodule
