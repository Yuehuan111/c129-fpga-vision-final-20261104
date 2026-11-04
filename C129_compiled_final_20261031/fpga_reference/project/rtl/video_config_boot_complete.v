// Preserve the original C97 ROM exactly and append an identity projective H.
module video_config_boot_complete #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720
)(
 input wire clk,rst_n,output wire valid,output wire[5:0] addr,
 output wire[31:0] data,input wire ready,output wire commit,
 input wire commit_ready,output wire boot_done
);
 wire original_valid,original_commit,original_done;
 wire[4:0] original_addr;
 wire[31:0] original_data;
 reg[3:0] matrix_index;
 reg begin_sent,matrix_sent,complete;
 video_config_boot #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) original(
  .clk(clk),.rst_n(rst_n),.valid(original_valid),.addr(original_addr),
  .data(original_data),.ready(ready&&begin_sent&&!original_commit&&!complete),
  .commit(original_commit),.commit_ready(matrix_sent&&commit_ready),.boot_done(original_done));
 assign valid=rst_n&&!complete&&(!begin_sent||original_valid||(original_commit&&!matrix_sent));
 assign addr=!begin_sent?6'h3f:original_valid?{1'b0,original_addr}:6'h20+matrix_index;
 assign data=!begin_sent?32'h434f4d50:original_valid?original_data:
             ((matrix_index==0||matrix_index==4||matrix_index==8)?32'h00010000:32'b0);
 assign commit=rst_n&&!complete&&original_commit&&matrix_sent;
 assign boot_done=complete;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin begin_sent<=0;matrix_index<=0;matrix_sent<=0;complete<=0;end
 else begin
  if(!begin_sent&&ready) begin_sent<=1;
  if(original_commit&&!matrix_sent&&ready) begin
   if(matrix_index==8) matrix_sent<=1;else matrix_index<=matrix_index+1'b1;
  end
  if(commit&&commit_ready) complete<=1;
 end
endmodule
