// Experimental bounded luma sharpening. Pixel operations execute in FPGA.
// One full raster-line delay, no multipliers, no frame buffer. The raw scan
// observation and metadata pixels must enter with bypass=1.
module detail_enhance_rgb565 #(
    parameter LINE_CLOCKS=1650,
    parameter LIMIT=8,
    parameter NOISE_RANGE=12,
    parameter LAPLACE_THRESHOLD=8
)(
    input wire pix_clk, input wire rst_n,
    input wire hs, input wire vs, input wire de,
    input wire [15:0] rgb565, input wire bypass,
    input wire config_valid, input wire config_enable,
    output reg out_hs, output reg out_vs, output reg out_de,
    output reg [15:0] out_rgb565,
    output reg status_valid, output reg applied_enable
);
    function integer bits_for;
        input integer n; integer a;
        begin a=n-1;bits_for=0;while(a>0)begin bits_for=bits_for+1;a=a>>1;end
            if(bits_for<1)bits_for=1;end
    endfunction
    function [7:0] luma;
        input [15:0] p;reg[9:0] s;
        begin s={2'b0,p[15:11],p[15:13]}+{1'b0,p[10:5],p[10:9],1'b0}+
                {2'b0,p[4:0],p[4:2]};luma=s[9:2];end
    endfunction
    function [7:0] min2;input[7:0]a,b;begin min2=a<b?a:b;end endfunction
    function [7:0] max2;input[7:0]a,b;begin max2=a>b?a:b;end endfunction
    function [7:0] add_sat;
        input[7:0]c;input signed[9:0]d;reg signed[10:0]v;
        begin v=$signed({3'b0,c})+d;
            if(v<0)add_sat=0;else if(v>255)add_sat=255;else add_sat=v[7:0];end
    endfunction
    localparam AW=bits_for(LINE_CLOCKS);
    // [20]=enable,[19]=bypass,[18]=vs,[17]=hs,[16]=de,[15:0]=RGB.
    (* syn_ramstyle="block_ram" *) reg [20:0] row_rgb[0:LINE_CLOCKS-1];
    // [9]=bypass,[8]=de,[7:0]=luma. It holds the row preceding row_rgb.
    (* syn_ramstyle="block_ram" *) reg [9:0] row_y[0:LINE_CLOCKS-1];
    reg[AW-1:0] addr,addr0;
    reg[1:0] filled_rows;
    reg pending_enable,vs_d;
    wire boundary=vs&&!vs_d&&!de;
    wire selected=config_valid?config_enable:pending_enable;
    reg[20:0] row0;
    reg[9:0] south0;
    reg valid0,north_valid0;
    reg[20:0] center1;
    reg[9:0] north1,south1;
    reg[7:0] y1;
    reg[20:0] center_d;
    reg[9:0] north_d,south_d;
    reg[7:0] y_d,west_y;
    reg west_de,west_bypass;
    reg[20:0] center2,center3;
    reg signed[10:0] lap2;
    reg[7:0] min2_y,max2_y,y2;
    reg eligible2;
    reg signed[9:0] delta3;
    reg signed[11:0] rounded_detail;
    reg signed[9:0] bounded_detail;
    reg signed[10:0] candidate_y;
    reg[7:0] new_r,new_g,new_b;

    always @* begin
        // Symmetric round-to-nearest magnitude; no negative-shift bias.
        if(lap2<0) rounded_detail=-(((-$signed({lap2[10],lap2}))+12'sd4)>>>3);
        else rounded_detail=($signed({lap2[10],lap2})+12'sd4)>>>3;
        if(rounded_detail>LIMIT) bounded_detail=LIMIT;
        else if(rounded_detail<-LIMIT) bounded_detail=-LIMIT;
        else bounded_detail=rounded_detail;
        candidate_y=$signed({3'b0,y2})+bounded_detail;
        if(candidate_y<$signed({3'b0,min2_y}))bounded_detail=$signed({2'b0,min2_y})-$signed({2'b0,y2});
        if(candidate_y>$signed({3'b0,max2_y}))bounded_detail=$signed({2'b0,max2_y})-$signed({2'b0,y2});
        new_r=add_sat({center3[15:11],center3[15:13]},delta3);
        new_g=add_sat({center3[10:5],center3[10:9]},delta3);
        new_b=add_sat({center3[4:0],center3[4:2]},delta3);
    end

    // RAM ports stay outside reset branches to infer simple block memories.
    always @(posedge pix_clk) begin
        row0<=row_rgb[addr];
        row_rgb[addr]<={applied_enable,bypass,vs,hs,de,rgb565};
        north1<=row_y[addr0];
        row_y[addr0]<={row0[19],row0[16]&&valid0,luma(row0[15:0])};
    end
    always @(posedge pix_clk or negedge rst_n) begin
        if(!rst_n)begin
            addr<=0;addr0<=0;filled_rows<=0;pending_enable<=0;applied_enable<=0;vs_d<=0;status_valid<=0;
            south0<=0;valid0<=0;north_valid0<=0;center1<=0;south1<=0;y1<=0;
            center_d<=0;north_d<=0;south_d<=0;y_d<=0;west_y<=0;west_de<=0;west_bypass<=0;
            center2<=0;center3<=0;lap2<=0;min2_y<=0;max2_y<=0;y2<=0;eligible2<=0;delta3<=0;
            out_hs<=0;out_vs<=0;out_de<=0;out_rgb565<=0;
        end else begin
            vs_d<=vs;status_valid<=0;
            if(config_valid)pending_enable<=config_enable;
            if(boundary)begin applied_enable<=selected;status_valid<=1;end
            addr0<=addr;
            if(addr==LINE_CLOCKS-1)begin addr<=0;if(filled_rows<2)filled_rows<=filled_rows+1'b1;end
            else addr<=addr+1'b1;
            valid0<=filled_rows!=0;north_valid0<=filled_rows==2;
            south0<={bypass,de,luma(rgb565)};
            center1<=valid0?row0:21'd0;
            south1<=south0;y1<=luma(row0[15:0]);
            center_d<=center1;
            north_d<=north_valid0?north1:10'd0;
            south_d<=south1;y_d<=y1;
            west_y<=y_d;west_de<=center_d[16];west_bypass<=center_d[19];
            center2<=center_d;y2<=y_d;
            lap2<=$signed({1'b0,y_d,2'b0})-$signed({3'b0,north_d[7:0]})-
                  $signed({3'b0,south_d[7:0]})-$signed({3'b0,west_y})-$signed({3'b0,y1});
            min2_y<=min2(y_d,min2(min2(north_d[7:0],south_d[7:0]),min2(west_y,y1)));
            max2_y<=max2(y_d,max2(max2(north_d[7:0],south_d[7:0]),max2(west_y,y1)));
            eligible2<=center_d[20]&&center_d[16]&&!center_d[19]&&center1[16]&&!center1[19]&&
                       west_de&&!west_bypass&&north_d[8]&&!north_d[9]&&south_d[8]&&!south_d[9];
            center3<=center2;
            if(eligible2&&(max2_y-min2_y>=NOISE_RANGE)&&
               (lap2>LAPLACE_THRESHOLD||lap2<-LAPLACE_THRESHOLD))delta3<=bounded_detail;
            else delta3<=0;
            out_hs<=center3[17];out_vs<=center3[18];out_de<=center3[16];
            if(delta3==0||!center3[16])out_rgb565<=center3[15:0];
            else out_rgb565<={new_r[7:3],new_g[7:2],new_b[7:3]};
        end
    end
endmodule
