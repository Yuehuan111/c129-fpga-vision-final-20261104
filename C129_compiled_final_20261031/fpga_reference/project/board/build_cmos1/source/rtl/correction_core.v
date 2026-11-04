// Frame-atomic Brown mapping + normalized 2D roll + bilinear RGB565 sampling.
// Memory read addresses are PIXEL indices, not bytes or DDR words.
// One outstanding read; responses may have arbitrary latency. Output may stall.
// Performance is intentionally measured, not assumed to meet video frame rate.
module correction_core #(
    parameter IMG_WIDTH=1280, IMG_HEIGHT=720
)(
    input wire clk, rst_n,
    input wire cfg_valid, input wire [4:0] cfg_addr,
    input wire [31:0] cfg_data, input wire cfg_commit,
    output wire cfg_ready, cfg_commit_ready,
    input wire start, input wire [31:0] frame_id,
    output wire busy, output reg done, output reg error,
    output reg [15:0] config_version,
    output wire rd_valid, input wire rd_ready, output wire [31:0] rd_addr,
    input wire rd_rsp_valid, output wire rd_rsp_ready, input wire [15:0] rd_rsp_data,
    output wire out_valid, input wire out_ready,
    output reg [15:0] out_rgb565, output reg [15:0] out_x, out_y,
    output reg out_last
);
    localparam IDLE=0, INIT=1, MAP=2, READ=3, RESPONSE=4, CALC=5, EMIT=6, PRODUCTS=7, SUMS=8, GAIN=9, OFFSET=10, PACK=11, PAIR_SUMS=12;
    reg [3:0] state;
    reg [31:0] shadow[0:15], pending[0:15], active[0:15];
    reg [15:0] written, pending_version;
    reg pending_valid;
    integer i,j;
    assign cfg_ready = rst_n && !cfg_commit;
    assign cfg_commit_ready = rst_n && (&written);
    assign busy = rst_n && state!=IDLE;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            written<=0; pending_version<=0; pending_valid<=0;
            for(i=0;i<16;i=i+1) begin shadow[i]<=0;pending[i]<=0;end
        end else begin
            if (cfg_valid && cfg_ready && cfg_addr<16) begin
                shadow[cfg_addr]<=cfg_data; written[cfg_addr]<=1;
            end
            if (cfg_commit && cfg_commit_ready) begin
                for(i=0;i<16;i=i+1) pending[i]<=shadow[i];
                written<=0; pending_version<=pending_version+1'b1; pending_valid<=1;
            end
        end
    end
    wire map_valid,map_done,map_error,map_last;
    wire [38:0] map_entry;
    brown_mapper #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) mapper(
        .clk(clk),.rst_n(rst_n),.start(state==INIT),
        .cfg_fx_src(active[0]),.cfg_fy_src(active[1]),.cfg_cx_src(active[2]),.cfg_cy_src(active[3]),
        .cfg_cx_out(active[4]),.cfg_cy_out(active[5]),.cfg_inv_fx_out(active[6]),.cfg_inv_fy_out(active[7]),
        .cfg_k1(active[8]),.cfg_k2(active[9]),.cfg_p1(active[10]),.cfg_p2(active[11]),
        .cfg_cos(active[12]),.cfg_sin(active[13]),
        .map_ready(state==MAP),.busy(),.done(map_done),.map_valid(map_valid),
        .map_entry(map_entry),.map_last(map_last),.error(map_error)
    );
    reg [10:0] x0,x1,y0,y1;
    reg [7:0] fx,fy;
    reg [1:0] neighbor;
    reg [15:0] pixels[0:3];
    wire [31:0] row0 = y0*IMG_WIDTH;
    wire [31:0] row1 = y1*IMG_WIDTH;
    assign rd_addr = (neighbor[1] ? row1 : row0)+(neighbor[0] ? x1 : x0);
    assign rd_valid = rst_n && state==READ;
    assign rd_rsp_ready = rst_n && state==RESPONSE;
    assign out_valid = rst_n && state==EMIT;
    // Each arithmetic layer has its own register: weights -> products -> sums
    // -> gain -> offset -> saturate. No input-to-output multiply/add cascade.
    wire [8:0] wx0=9'd256-{1'b0,fx}, wy0=9'd256-{1'b0,fy};
    reg [17:0] weights[0:3];
    reg [25:0] products_r[0:3],products_g[0:3],products_b[0:3];
    reg [26:0] pair_r0,pair_r1,pair_g0,pair_g1,pair_b0,pair_b1;
    reg [27:0] sum_r,sum_g,sum_b;
    reg signed [63:0] gain_r,gain_g,gain_b,adjust_r,adjust_g,adjust_b;
    // ADS Synthesis User Guide 5.7: map these 9x32 signed gain multiplies
    // to LUTs. The automatic multi-APM cascade had a failing short PI path
    // on CMOS1. Keep the same 64-bit arithmetic and the existing GAIN stage.
    wire signed [63:0] gain_product_r /* synthesis syn_dspstyle = "logic" */;
    wire signed [63:0] gain_product_g /* synthesis syn_dspstyle = "logic" */;
    wire signed [63:0] gain_product_b /* synthesis syn_dspstyle = "logic" */;
    assign gain_product_r=$signed({1'b0,sum_r[23:16]})*$signed(active[14]);
    assign gain_product_g=$signed({1'b0,sum_g[23:16]})*$signed(active[14]);
    assign gain_product_b=$signed({1'b0,sum_b[23:16]})*$signed(active[14]);
    function [7:0] red; input [15:0] p;begin red={p[15:11],p[15:13]};end endfunction
    function [7:0] green; input [15:0] p;begin green={p[10:5],p[10:9]};end endfunction
    function [7:0] blue; input [15:0] p;begin blue={p[4:0],p[4:2]};end endfunction
    function [7:0] saturate;
      input signed [63:0] value;
      begin if(value<0) saturate=0;else if(value>255) saturate=255;else saturate=value[7:0];end
    endfunction
    wire [7:0] r8=saturate(adjust_r),g8=saturate(adjust_g),b8=saturate(adjust_b);
    reg [31:0] accepted_frame_id;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;done<=0;error<=0;config_version<=0;
            out_x<=0;out_y<=0;out_last<=0;out_rgb565<=0;
            x0<=0;x1<=0;y0<=0;y1<=0;fx<=0;fy<=0;neighbor<=0;accepted_frame_id<=0;
            for(j=0;j<16;j=j+1) active[j]<=0;
            for(j=0;j<4;j=j+1) begin pixels[j]<=0;weights[j]<=0;
              products_r[j]<=0;products_g[j]<=0;products_b[j]<=0;end
            pair_r0<=0;pair_r1<=0;pair_g0<=0;pair_g1<=0;pair_b0<=0;pair_b1<=0;
            sum_r<=0;sum_g<=0;sum_b<=0;gain_r<=0;gain_g<=0;gain_b<=0;
            adjust_r<=0;adjust_g<=0;adjust_b<=0;
        end else begin
            done<=0;
            if(state!=IDLE && state!=INIT && map_error) error<=1;
            case(state)
                IDLE: if(start && pending_valid && !cfg_commit) begin
                    for(j=0;j<16;j=j+1) active[j]<=pending[j];
                    config_version<=pending_version; accepted_frame_id<=frame_id;
                    out_x<=0;out_y<=0;out_last<=0;error<=0;state<=INIT;
                end
                INIT: state<=MAP;
                MAP: if(map_valid) begin
                    out_last<=map_last;
                    if(!map_entry[38]) begin out_rgb565<=0;state<=EMIT;end
                    else begin
                        x0<=map_entry[37:27]; y0<=map_entry[18:8];
                        x1<=map_entry[37:27]==IMG_WIDTH-1 ? map_entry[37:27] : map_entry[37:27]+1'b1;
                        y1<=map_entry[18:8]==IMG_HEIGHT-1 ? map_entry[18:8] : map_entry[18:8]+1'b1;
                        fx<=map_entry[26:19]; fy<=map_entry[7:0];neighbor<=0;state<=READ;
                    end
                end
                READ: if(rd_ready) state<=RESPONSE;
                RESPONSE: if(rd_rsp_valid) begin
                    pixels[neighbor]<=rd_rsp_data;
                    if(neighbor==3) state<=CALC;
                    else begin neighbor<=neighbor+1'b1;state<=READ;end
                end
                CALC: begin
                  weights[0]<=wx0*wy0;weights[1]<={1'b0,fx}*wy0;
                  weights[2]<=wx0*{1'b0,fy};weights[3]<={1'b0,fx}*{1'b0,fy};state<=PRODUCTS;
                end
                PRODUCTS: begin
                  for(j=0;j<4;j=j+1) begin
                    products_r[j]<=red(pixels[j])*weights[j];
                    products_g[j]<=green(pixels[j])*weights[j];
                    products_b[j]<=blue(pixels[j])*weights[j];
                  end
                  state<=PAIR_SUMS;
                end
                PAIR_SUMS: begin
                  pair_r0<=products_r[0]+products_r[1];pair_r1<=products_r[2]+products_r[3];
                  pair_g0<=products_g[0]+products_g[1];pair_g1<=products_g[2]+products_g[3];
                  pair_b0<=products_b[0]+products_b[1];pair_b1<=products_b[2]+products_b[3];state<=SUMS;
                end
                SUMS: begin
                  sum_r<=pair_r0+pair_r1+28'd32768;
                  sum_g<=pair_g0+pair_g1+28'd32768;
                  sum_b<=pair_b0+pair_b1+28'd32768;state<=GAIN;
                end
                GAIN: begin
                  gain_r<=gain_product_r;
                  gain_g<=gain_product_g;
                  gain_b<=gain_product_b;state<=OFFSET;
                end
                OFFSET: begin
                  adjust_r<=((gain_r+64'sd128)>>>8)+$signed(active[15]);
                  adjust_g<=((gain_g+64'sd128)>>>8)+$signed(active[15]);
                  adjust_b<=((gain_b+64'sd128)>>>8)+$signed(active[15]);state<=PACK;
                end
                PACK: begin out_rgb565<={r8[7:3],g8[7:2],b8[7:3]};state<=EMIT;end
                EMIT: if(out_ready) begin
                    if(out_last) begin done<=1;state<=IDLE;end
                    else begin
                        if(out_x==IMG_WIDTH-1) begin out_x<=0;out_y<=out_y+1'b1;end
                        else out_x<=out_x+1'b1;
                        state<=MAP;
                    end
                end
                default: begin state<=IDLE;error<=1;end
            endcase
        end
    end
endmodule
