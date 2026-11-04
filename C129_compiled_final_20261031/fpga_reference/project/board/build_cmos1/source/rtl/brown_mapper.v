// Brown model: destination (undistorted) pixel -> source (distorted) coordinate.
// All cfg_* ports are signed Q16.16 and are captured only on an idle start.
// Dimensions 1..2048. Output {valid, xQ11.8, yQ11.8}; camera roll in normalized coordinates.
// One shared signed 32x32 multiplier; every multiplication truncates with >>>16.
// A 32-bit arithmetic overflow invalidates this pixel and sets sticky run error.
// Source coordinates outside the image produce zero entries without an error.
// map_valid, map_entry and map_last remain stable until map_ready accepts them.
// Q16.16 truncation/overflow rules retained; six rotation steps precede Brown distortion.
module brown_mapper #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [31:0] cfg_fx_src,
    input  wire signed [31:0] cfg_fy_src,
    input  wire signed [31:0] cfg_cx_src,
    input  wire signed [31:0] cfg_cy_src,
    input  wire signed [31:0] cfg_cx_out,
    input  wire signed [31:0] cfg_cy_out,
    input  wire signed [31:0] cfg_inv_fx_out,
    input  wire signed [31:0] cfg_inv_fy_out,
    input  wire signed [31:0] cfg_k1,
    input  wire signed [31:0] cfg_k2,
    input  wire signed [31:0] cfg_p1,
    input  wire signed [31:0] cfg_p2,
    input wire signed [31:0] cfg_cos, cfg_sin,
    input  wire               map_ready,
    output wire               busy,
    output reg                done,
    output reg                map_valid,
    output reg         [38:0] map_entry,
    output reg                map_last,
    output reg                error
);

localparam signed [31:0] Q_ONE = 32'sd65536;
localparam signed [31:0] MAX_X_Q16 = (IMG_WIDTH-1) * 65536;
localparam signed [31:0] MAX_Y_Q16 = (IMG_HEIGHT-1) * 65536;
localparam [24:0] MAX_X_Q8 = (IMG_WIDTH-1) * 256;
localparam [24:0] MAX_Y_Q8 = (IMG_HEIGHT-1) * 256;
localparam [5:0] S_IDLE = 0, S_PACK = 40, S_SEND = 41;
localparam [1:0] OP_MUL = 0, OP_ADD = 1, OP_SUB = 2, OP_NONE = 3;

reg [5:0] step;
reg [15:0] pixel_x, pixel_y;
reg pixel_invalid;
reg signed [31:0] fx_src, fy_src, cx_src, cy_src;
reg signed [31:0] cx_out, cy_out, inv_fx_out, inv_fy_out;
reg signed [31:0] k1, k2, p1, p2;
reg signed [31:0] cos_a, sin_a, rcx, rsy, rsx, rcy;
reg signed [31:0] du, dv, x, y, x2, y2, xy, r2, r4;
reg signed [31:0] k1r2, k2r4, radial_a, radial, xr, yr;
reg signed [31:0] p1xy, p2xy, twice_x2, twice_y2, tx, ty;
reg signed [31:0] p2tx, p1ty, twice_p1xy, twice_p2xy;
reg signed [31:0] xd_a, xd, yd_a, yd;
reg signed [31:0] us_scaled, vs_scaled, source_u, source_v;
reg signed [31:0] operand_a, operand_b;
reg [1:0] operation;

assign busy = rst_n && (step != S_IDLE);

// Operand selection, arithmetic, overflow selection and destination writes are
// registered separately. This breaks the old step -> mux -> multiply -> CE path.
reg [1:0] phase;
reg signed [31:0] selected_a, selected_b;
reg [1:0] selected_op;
reg signed [63:0] multiply_full;
reg signed [32:0] add_full;
reg signed [31:0] arithmetic_result;
reg arithmetic_overflow;
wire signed [63:0] multiply_scaled = multiply_full >>> 16;
wire signed [32:0] a_extended = {selected_a[31], selected_a};
wire signed [32:0] b_extended = {selected_b[31], selected_b};

// Quantization is used only after the signed Q16.16 bounds check passes.
// Widen the rounding addition so even an invalid input cannot wrap this wire.
wire [32:0] rounded_u = {1'b0, source_u} + 33'd128;
wire [32:0] rounded_v = {1'b0, source_v} + 33'd128;
wire [24:0] u_q8 = rounded_u[32:8];
wire [24:0] v_q8 = rounded_v[32:8];
wire source_inside = (source_u >= 32'sd0) && (source_v >= 32'sd0) &&
                     (source_u <= MAX_X_Q16) && (source_v <= MAX_Y_Q16);
wire quantized_inside = (u_q8 <= MAX_X_Q8) && (v_q8 <= MAX_Y_Q8);

// One operation per step. Each result is stored before the next step executes.
always @* begin
    operation = OP_NONE;
    operand_a = 0;
    operand_b = 0;
    case (step)
        6'd1:  begin operation=OP_SUB; operand_a=$signed({pixel_x,16'd0}); operand_b=cx_out; end
        6'd2:  begin operation=OP_SUB; operand_a=$signed({pixel_y,16'd0}); operand_b=cy_out; end
        6'd3:  begin operation=OP_MUL; operand_a=du; operand_b=inv_fx_out; end
        6'd4:  begin operation=OP_MUL; operand_a=dv; operand_b=inv_fy_out; end
        6'd5: begin operation=OP_MUL; operand_a=x; operand_b=cos_a; end
        6'd6: begin operation=OP_MUL; operand_a=y; operand_b=sin_a; end
        6'd7: begin operation=OP_MUL; operand_a=x; operand_b=sin_a; end
        6'd8: begin operation=OP_MUL; operand_a=y; operand_b=cos_a; end
        6'd9: begin operation=OP_SUB; operand_a=rcx; operand_b=rsy; end
        6'd10: begin operation=OP_ADD; operand_a=rsx; operand_b=rcy; end
        6'd11:  begin operation=OP_MUL; operand_a=x; operand_b=x; end
        6'd12:  begin operation=OP_MUL; operand_a=y; operand_b=y; end
        6'd13:  begin operation=OP_MUL; operand_a=x; operand_b=y; end
        6'd14:  begin operation=OP_ADD; operand_a=x2; operand_b=y2; end
        6'd15:  begin operation=OP_MUL; operand_a=r2; operand_b=r2; end
        6'd16: begin operation=OP_MUL; operand_a=k1; operand_b=r2; end
        6'd17: begin operation=OP_MUL; operand_a=k2; operand_b=r4; end
        6'd18: begin operation=OP_ADD; operand_a=Q_ONE; operand_b=k1r2; end
        6'd19: begin operation=OP_ADD; operand_a=radial_a; operand_b=k2r4; end
        6'd20: begin operation=OP_MUL; operand_a=x; operand_b=radial; end
        6'd21: begin operation=OP_MUL; operand_a=y; operand_b=radial; end
        6'd22: begin operation=OP_MUL; operand_a=p1; operand_b=xy; end
        6'd23: begin operation=OP_MUL; operand_a=p2; operand_b=xy; end
        6'd24: begin operation=OP_ADD; operand_a=x2; operand_b=x2; end
        6'd25: begin operation=OP_ADD; operand_a=y2; operand_b=y2; end
        6'd26: begin operation=OP_ADD; operand_a=r2; operand_b=twice_x2; end
        6'd27: begin operation=OP_ADD; operand_a=r2; operand_b=twice_y2; end
        6'd28: begin operation=OP_MUL; operand_a=p2; operand_b=tx; end
        6'd29: begin operation=OP_MUL; operand_a=p1; operand_b=ty; end
        6'd30: begin operation=OP_ADD; operand_a=p1xy; operand_b=p1xy; end
        6'd31: begin operation=OP_ADD; operand_a=p2xy; operand_b=p2xy; end
        6'd32: begin operation=OP_ADD; operand_a=xr; operand_b=twice_p1xy; end
        6'd33: begin operation=OP_ADD; operand_a=xd_a; operand_b=p2tx; end
        6'd34: begin operation=OP_ADD; operand_a=yr; operand_b=p1ty; end
        6'd35: begin operation=OP_ADD; operand_a=yd_a; operand_b=twice_p2xy; end
        6'd36: begin operation=OP_MUL; operand_a=fx_src; operand_b=xd; end
        6'd37: begin operation=OP_MUL; operand_a=fy_src; operand_b=yd; end
        6'd38: begin operation=OP_ADD; operand_a=us_scaled; operand_b=cx_src; end
        6'd39: begin operation=OP_ADD; operand_a=vs_scaled; operand_b=cy_src; end
        default: begin end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        step <= S_IDLE;
        phase<=0;selected_a<=0;selected_b<=0;selected_op<=OP_NONE;
        multiply_full<=0;add_full<=0;arithmetic_result<=0;arithmetic_overflow<=0;
        pixel_x <= 0;
        pixel_y <= 0;
        pixel_invalid <= 0;
        done <= 0;
        map_valid <= 0;
        map_entry <= 0;
        map_last <= 0;
        error <= 0;
        fx_src <= 0; fy_src <= 0; cx_src <= 0; cy_src <= 0;
        cx_out <= 0; cy_out <= 0; inv_fx_out <= 0; inv_fy_out <= 0;
        k1 <= 0; k2 <= 0; p1 <= 0; p2 <= 0;
        cos_a<=65536; sin_a<=0; rcx<=0; rsy<=0; rsx<=0; rcy<=0;
        du <= 0; dv <= 0; x <= 0; y <= 0; x2 <= 0; y2 <= 0; xy <= 0;
        r2 <= 0; r4 <= 0; k1r2 <= 0; k2r4 <= 0; radial_a <= 0; radial <= 0;
        xr <= 0; yr <= 0; p1xy <= 0; p2xy <= 0; twice_x2 <= 0; twice_y2 <= 0;
        tx <= 0; ty <= 0; p2tx <= 0; p1ty <= 0; twice_p1xy <= 0; twice_p2xy <= 0;
        xd_a <= 0; xd <= 0; yd_a <= 0; yd <= 0;
        us_scaled <= 0; vs_scaled <= 0; source_u <= 0; source_v <= 0;
    end else begin
        done <= 0;
        if (step == S_IDLE) begin
            if (start) begin
                fx_src <= cfg_fx_src; fy_src <= cfg_fy_src;
                cx_src <= cfg_cx_src; cy_src <= cfg_cy_src;
                cx_out <= cfg_cx_out; cy_out <= cfg_cy_out;
                inv_fx_out <= cfg_inv_fx_out; inv_fy_out <= cfg_inv_fy_out;
                k1 <= cfg_k1; k2 <= cfg_k2; p1 <= cfg_p1; p2 <= cfg_p2;
                cos_a <= cfg_cos; sin_a <= cfg_sin;
                pixel_x <= 0;
                pixel_y <= 0;
                pixel_invalid <= 0;
                error <= 0;
                step <= 6'd1;
            end
        end else if (step == S_PACK) begin
            if (!pixel_invalid && source_inside && quantized_inside)
                map_entry <= {1'b1, u_q8[18:0], v_q8[18:0]};
            else
                map_entry <= 39'd0;
            map_valid <= 1;
            map_last <= (pixel_x == IMG_WIDTH-1) && (pixel_y == IMG_HEIGHT-1);
            step <= S_SEND;
        end else if (step == S_SEND) begin
            if (map_valid && map_ready) begin
                map_valid <= 0;
                map_last <= 0;
                if (map_last) begin
                    done <= 1;
                    step <= S_IDLE;
                end else begin
                    if (pixel_x == IMG_WIDTH-1) begin
                        pixel_x <= 0;
                        pixel_y <= pixel_y + 1'b1;
                    end else
                        pixel_x <= pixel_x + 1'b1;
                    pixel_invalid <= 0;
                    step <= 6'd1;
                end
            end
        end else if (operation != OP_NONE) begin
            if(phase==0) begin
                selected_a<=operand_a;selected_b<=operand_b;selected_op<=operation;phase<=1;
            end else if(phase==1) begin
                multiply_full<=selected_a*selected_b;
                add_full<=(selected_op==OP_SUB)?a_extended-b_extended:a_extended+b_extended;
                phase<=2;
            end else if(phase==2) begin
                arithmetic_result<=(selected_op==OP_MUL)?multiply_scaled[31:0]:add_full[31:0];
                arithmetic_overflow<=(selected_op==OP_MUL)?
                    (multiply_scaled[63:32]!={32{multiply_scaled[31]}}):(add_full[32]!=add_full[31]);
                phase<=3;
            end else begin
            phase<=0;
            if (arithmetic_overflow) begin
                pixel_invalid <= 1;
                error <= 1;
                step <= S_PACK;
            end else begin
                case (step)
                    6'd1: du <= arithmetic_result;
                    6'd2: dv <= arithmetic_result;
                    6'd3: x <= arithmetic_result;
                    6'd4: y <= arithmetic_result;
                    6'd5: rcx <= arithmetic_result;
                    6'd6: rsy <= arithmetic_result;
                    6'd7: rsx <= arithmetic_result;
                    6'd8: rcy <= arithmetic_result;
                    6'd9: x <= arithmetic_result;
                    6'd10: y <= arithmetic_result;
                    6'd11: x2 <= arithmetic_result;
                    6'd12: y2 <= arithmetic_result;
                    6'd13: xy <= arithmetic_result;
                    6'd14: r2 <= arithmetic_result;
                    6'd15: r4 <= arithmetic_result;
                    6'd16: k1r2 <= arithmetic_result;
                    6'd17: k2r4 <= arithmetic_result;
                    6'd18: radial_a <= arithmetic_result;
                    6'd19: radial <= arithmetic_result;
                    6'd20: xr <= arithmetic_result;
                    6'd21: yr <= arithmetic_result;
                    6'd22: p1xy <= arithmetic_result;
                    6'd23: p2xy <= arithmetic_result;
                    6'd24: twice_x2 <= arithmetic_result;
                    6'd25: twice_y2 <= arithmetic_result;
                    6'd26: tx <= arithmetic_result;
                    6'd27: ty <= arithmetic_result;
                    6'd28: p2tx <= arithmetic_result;
                    6'd29: p1ty <= arithmetic_result;
                    6'd30: twice_p1xy <= arithmetic_result;
                    6'd31: twice_p2xy <= arithmetic_result;
                    6'd32: xd_a <= arithmetic_result;
                    6'd33: xd <= arithmetic_result;
                    6'd34: yd_a <= arithmetic_result;
                    6'd35: yd <= arithmetic_result;
                    6'd36: us_scaled <= arithmetic_result;
                    6'd37: vs_scaled <= arithmetic_result;
                    6'd38: source_u <= arithmetic_result;
                    6'd39: source_v <= arithmetic_result;
                    default: begin end
                endcase
                step <= step + 1'b1;
            end
            end
        end else begin
            phase<=0;
            // Recover a corrupted FSM without presenting a spurious map word.
            step <= S_IDLE;
            map_valid <= 0;
            map_last <= 0;
            error <= 1;
        end
    end
end

endmodule
