// 1080p30 raster at 74.25 MHz: 2200 x 1125, positive H/V sync.
module video_timing_1080 #(
 parameter H_ACTIVE=1920,H_FRONT=88,H_SYNC=44,H_BACK=148,
 parameter V_ACTIVE=1080,V_FRONT=4,V_SYNC=5,V_BACK=36
)(input wire clk,rst_n,output wire hs,vs,de,output wire[15:0] x,y);
 localparam HT=H_ACTIVE+H_FRONT+H_SYNC+H_BACK;
 localparam VT=V_ACTIVE+V_FRONT+V_SYNC+V_BACK;
 reg[15:0] hc,vc;
 // Sync first leaves a back porch in which the display fetches its first row.
 assign hs=hc<H_SYNC;assign vs=vc<V_SYNC;
 assign de=hc>=H_SYNC+H_BACK && hc<H_SYNC+H_BACK+H_ACTIVE &&
           vc>=V_SYNC+V_BACK && vc<V_SYNC+V_BACK+V_ACTIVE;
 assign x=hc-H_SYNC-H_BACK;assign y=vc-V_SYNC-V_BACK;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin hc<=0;vc<=0;end
 else if(hc==HT-1) begin hc<=0;vc<=vc==VT-1?0:vc+1'b1;end
 else hc<=hc+1'b1;
endmodule
