// Adaptive Gamma for RGB565. All state and arithmetic are in the pixel domain.
// Each pixel and HS/VS/DE pass through exactly one register stage.
// Static LUT entries are round(MAX * (code/MAX)**gamma), gamma=0.7 or 1.4.
// No runtime floating point, power, divider, external ROM, or host frame map.
module adaptive_gamma_rgb565 #(
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter DEFAULT_MODE = 1,
    parameter CONFIRM_FRAMES = 2
)(
    input wire pix_clk,
    input wire rst_n,
    input wire hs,
    input wire vs,
    input wire de,
    input wire [15:0] rgb565,
    input wire config_valid,
    input wire [1:0] config_mode,
    output reg out_hs,
    output reg out_vs,
    output reg out_de,
    output reg [15:0] out_rgb565,
    output reg status_valid,
    output reg [3:0] status_data
);
    localparam [31:0] FRAME_PIXELS = IMG_WIDTH * IMG_HEIGHT;
    localparam [31:0] MIN_NONZERO = (FRAME_PIXELS + 3) / 4;
    localparam [31:0] CONFIRM_LIMIT = (CONFIRM_FRAMES < 1) ? 1 : CONFIRM_FRAMES;
    localparam [1:0] RESET_MODE = DEFAULT_MODE;
    localparam [1:0] RESET_GAMMA = (DEFAULT_MODE == 2) ? 2'd1 :
                                       (DEFAULT_MODE == 3) ? 2'd2 : 2'd0;

    reg vs_d;
    reg [1:0] pending_mode;
    reg [1:0] applied_mode;
    reg [1:0] gamma_id;
    reg [1:0] auto_gamma;
    reg [1:0] confirm_target;
    reg [31:0] confirm_count;
    reg [31:0] luma_sum;
    reg [31:0] pixel_count;
    reg [31:0] nonzero_count;

    wire [7:0] red8 = {rgb565[15:11], rgb565[15:13]};
    wire [7:0] green8 = {rgb565[10:5], rgb565[10:9]};
    wire [7:0] blue8 = {rgb565[4:0], rgb565[4:2]};
    wire [9:0] weighted_luma = {2'b00, red8} + {1'b0, green8, 1'b0} + {2'b00, blue8};
    wire [7:0] input_luma = weighted_luma[9:2];
    // Only a VS rising edge in blanking is a frame boundary. Malformed DE-high
    // edges neither apply a configuration nor certify a completed frame.
    wire frame_boundary = vs && !vs_d && !de;
    wire valid_frame = (pixel_count == FRAME_PIXELS) && (nonzero_count >= MIN_NONZERO);
    wire [63:0] sum_wide = {32'd0, luma_sum};
    // Zero RGB pixels are excluded from the mean (black-border/startup heuristic).
    wire [63:0] count_wide = {32'd0, nonzero_count};
    // Widen BEFORE shifts/additions, so 1280*720*192 cannot truncate.
    wire [63:0] threshold_64 = count_wide << 6;
    wire [63:0] threshold_80 = (count_wide << 6) + (count_wide << 4);
    wire [63:0] threshold_176 = (count_wide << 7) + (count_wide << 5) + (count_wide << 4);
    wire [63:0] threshold_192 = (count_wide << 7) + (count_wide << 6);

    reg [1:0] desired_auto;
    reg [1:0] next_auto;
    reg [1:0] next_confirm_target;
    reg [31:0] next_confirm_count;
    reg [1:0] next_mode;
    reg [1:0] next_gamma;

    // Auto statistics always use the original input, including in manual modes.
    // Invalid/incomplete/mostly-zero frames break the confirmation streak and
    // retain the last confirmed auto choice. This is a heuristic, not a general
    // DDR underflow detector.
    always @* begin
        desired_auto = auto_gamma;
        case (auto_gamma)
            2'd0: begin
                if (sum_wide < threshold_64) desired_auto = 2'd1;
                else if (sum_wide > threshold_192) desired_auto = 2'd2;
            end
            2'd1: if (sum_wide > threshold_80) desired_auto = 2'd0;
            2'd2: if (sum_wide < threshold_176) desired_auto = 2'd0;
            default: desired_auto = 2'd0;
        endcase
        next_auto = auto_gamma;
        next_confirm_target = confirm_target;
        next_confirm_count = confirm_count;
        if (!valid_frame || desired_auto == auto_gamma) begin
            next_confirm_target = auto_gamma;
            next_confirm_count = 32'd0;
        end else if (CONFIRM_LIMIT == 1) begin
            next_auto = desired_auto;
            next_confirm_target = desired_auto;
            next_confirm_count = 32'd0;
        end else if (confirm_target != desired_auto) begin
            next_confirm_target = desired_auto;
            next_confirm_count = 32'd1;
        end else if (confirm_count >= CONFIRM_LIMIT - 32'd1) begin
            next_auto = desired_auto;
            next_confirm_target = desired_auto;
            next_confirm_count = 32'd0;
        end else begin
            next_confirm_count = confirm_count + 32'd1;
        end
        // A mailbox notification received on this boundary is already safe in
        // this clock domain and can take effect for the immediately next frame.
        next_mode = config_valid ? config_mode : pending_mode;
        case (next_mode)
            2'd0: next_gamma = 2'd0;
            2'd1: next_gamma = next_auto;
            2'd2: next_gamma = 2'd1;
            default: next_gamma = 2'd2;
        endcase
    end

    function [4:0] gamma5;
        input [4:0] code;
        input [1:0] choice;
        begin
            case (choice)
                2'd1: begin
                    case (code)
                        5'd0: gamma5 = 5'd0;
                        5'd1: gamma5 = 5'd3;
                        5'd2: gamma5 = 5'd5;
                        5'd3: gamma5 = 5'd6;
                        5'd4: gamma5 = 5'd7;
                        5'd5: gamma5 = 5'd9;
                        5'd6: gamma5 = 5'd10;
                        5'd7: gamma5 = 5'd11;
                        5'd8: gamma5 = 5'd12;
                        5'd9: gamma5 = 5'd13;
                        5'd10: gamma5 = 5'd14;
                        5'd11: gamma5 = 5'd15;
                        5'd12: gamma5 = 5'd16;
                        5'd13: gamma5 = 5'd17;
                        5'd14: gamma5 = 5'd18;
                        5'd15: gamma5 = 5'd19;
                        5'd16: gamma5 = 5'd20;
                        5'd17: gamma5 = 5'd20;
                        5'd18: gamma5 = 5'd21;
                        5'd19: gamma5 = 5'd22;
                        5'd20: gamma5 = 5'd23;
                        5'd21: gamma5 = 5'd24;
                        5'd22: gamma5 = 5'd24;
                        5'd23: gamma5 = 5'd25;
                        5'd24: gamma5 = 5'd26;
                        5'd25: gamma5 = 5'd27;
                        5'd26: gamma5 = 5'd27;
                        5'd27: gamma5 = 5'd28;
                        5'd28: gamma5 = 5'd29;
                        5'd29: gamma5 = 5'd30;
                        5'd30: gamma5 = 5'd30;
                        5'd31: gamma5 = 5'd31;
                        default: gamma5 = code;
                    endcase
                end
                2'd2: begin
                    case (code)
                        5'd0: gamma5 = 5'd0;
                        5'd1: gamma5 = 5'd0;
                        5'd2: gamma5 = 5'd1;
                        5'd3: gamma5 = 5'd1;
                        5'd4: gamma5 = 5'd2;
                        5'd5: gamma5 = 5'd2;
                        5'd6: gamma5 = 5'd3;
                        5'd7: gamma5 = 5'd4;
                        5'd8: gamma5 = 5'd5;
                        5'd9: gamma5 = 5'd5;
                        5'd10: gamma5 = 5'd6;
                        5'd11: gamma5 = 5'd7;
                        5'd12: gamma5 = 5'd8;
                        5'd13: gamma5 = 5'd9;
                        5'd14: gamma5 = 5'd10;
                        5'd15: gamma5 = 5'd11;
                        5'd16: gamma5 = 5'd12;
                        5'd17: gamma5 = 5'd13;
                        5'd18: gamma5 = 5'd14;
                        5'd19: gamma5 = 5'd16;
                        5'd20: gamma5 = 5'd17;
                        5'd21: gamma5 = 5'd18;
                        5'd22: gamma5 = 5'd19;
                        5'd23: gamma5 = 5'd20;
                        5'd24: gamma5 = 5'd22;
                        5'd25: gamma5 = 5'd23;
                        5'd26: gamma5 = 5'd24;
                        5'd27: gamma5 = 5'd26;
                        5'd28: gamma5 = 5'd27;
                        5'd29: gamma5 = 5'd28;
                        5'd30: gamma5 = 5'd30;
                        5'd31: gamma5 = 5'd31;
                        default: gamma5 = code;
                    endcase
                end
                default: gamma5 = code;
            endcase
        end
    endfunction

    function [5:0] gamma6;
        input [5:0] code;
        input [1:0] choice;
        begin
            case (choice)
                2'd1: begin
                    case (code)
                        6'd0: gamma6 = 6'd0;
                        6'd1: gamma6 = 6'd3;
                        6'd2: gamma6 = 6'd6;
                        6'd3: gamma6 = 6'd7;
                        6'd4: gamma6 = 6'd9;
                        6'd5: gamma6 = 6'd11;
                        6'd6: gamma6 = 6'd12;
                        6'd7: gamma6 = 6'd14;
                        6'd8: gamma6 = 6'd15;
                        6'd9: gamma6 = 6'd16;
                        6'd10: gamma6 = 6'd17;
                        6'd11: gamma6 = 6'd19;
                        6'd12: gamma6 = 6'd20;
                        6'd13: gamma6 = 6'd21;
                        6'd14: gamma6 = 6'd22;
                        6'd15: gamma6 = 6'd23;
                        6'd16: gamma6 = 6'd24;
                        6'd17: gamma6 = 6'd25;
                        6'd18: gamma6 = 6'd26;
                        6'd19: gamma6 = 6'd27;
                        6'd20: gamma6 = 6'd28;
                        6'd21: gamma6 = 6'd29;
                        6'd22: gamma6 = 6'd30;
                        6'd23: gamma6 = 6'd31;
                        6'd24: gamma6 = 6'd32;
                        6'd25: gamma6 = 6'd33;
                        6'd26: gamma6 = 6'd34;
                        6'd27: gamma6 = 6'd35;
                        6'd28: gamma6 = 6'd36;
                        6'd29: gamma6 = 6'd37;
                        6'd30: gamma6 = 6'd37;
                        6'd31: gamma6 = 6'd38;
                        6'd32: gamma6 = 6'd39;
                        6'd33: gamma6 = 6'd40;
                        6'd34: gamma6 = 6'd41;
                        6'd35: gamma6 = 6'd42;
                        6'd36: gamma6 = 6'd43;
                        6'd37: gamma6 = 6'd43;
                        6'd38: gamma6 = 6'd44;
                        6'd39: gamma6 = 6'd45;
                        6'd40: gamma6 = 6'd46;
                        6'd41: gamma6 = 6'd47;
                        6'd42: gamma6 = 6'd47;
                        6'd43: gamma6 = 6'd48;
                        6'd44: gamma6 = 6'd49;
                        6'd45: gamma6 = 6'd50;
                        6'd46: gamma6 = 6'd51;
                        6'd47: gamma6 = 6'd51;
                        6'd48: gamma6 = 6'd52;
                        6'd49: gamma6 = 6'd53;
                        6'd50: gamma6 = 6'd54;
                        6'd51: gamma6 = 6'd54;
                        6'd52: gamma6 = 6'd55;
                        6'd53: gamma6 = 6'd56;
                        6'd54: gamma6 = 6'd57;
                        6'd55: gamma6 = 6'd57;
                        6'd56: gamma6 = 6'd58;
                        6'd57: gamma6 = 6'd59;
                        6'd58: gamma6 = 6'd59;
                        6'd59: gamma6 = 6'd60;
                        6'd60: gamma6 = 6'd61;
                        6'd61: gamma6 = 6'd62;
                        6'd62: gamma6 = 6'd62;
                        6'd63: gamma6 = 6'd63;
                        default: gamma6 = code;
                    endcase
                end
                2'd2: begin
                    case (code)
                        6'd0: gamma6 = 6'd0;
                        6'd1: gamma6 = 6'd0;
                        6'd2: gamma6 = 6'd1;
                        6'd3: gamma6 = 6'd1;
                        6'd4: gamma6 = 6'd1;
                        6'd5: gamma6 = 6'd2;
                        6'd6: gamma6 = 6'd2;
                        6'd7: gamma6 = 6'd3;
                        6'd8: gamma6 = 6'd4;
                        6'd9: gamma6 = 6'd4;
                        6'd10: gamma6 = 6'd5;
                        6'd11: gamma6 = 6'd5;
                        6'd12: gamma6 = 6'd6;
                        6'd13: gamma6 = 6'd7;
                        6'd14: gamma6 = 6'd8;
                        6'd15: gamma6 = 6'd8;
                        6'd16: gamma6 = 6'd9;
                        6'd17: gamma6 = 6'd10;
                        6'd18: gamma6 = 6'd11;
                        6'd19: gamma6 = 6'd12;
                        6'd20: gamma6 = 6'd13;
                        6'd21: gamma6 = 6'd14;
                        6'd22: gamma6 = 6'd14;
                        6'd23: gamma6 = 6'd15;
                        6'd24: gamma6 = 6'd16;
                        6'd25: gamma6 = 6'd17;
                        6'd26: gamma6 = 6'd18;
                        6'd27: gamma6 = 6'd19;
                        6'd28: gamma6 = 6'd20;
                        6'd29: gamma6 = 6'd21;
                        6'd30: gamma6 = 6'd22;
                        6'd31: gamma6 = 6'd23;
                        6'd32: gamma6 = 6'd24;
                        6'd33: gamma6 = 6'd25;
                        6'd34: gamma6 = 6'd27;
                        6'd35: gamma6 = 6'd28;
                        6'd36: gamma6 = 6'd29;
                        6'd37: gamma6 = 6'd30;
                        6'd38: gamma6 = 6'd31;
                        6'd39: gamma6 = 6'd32;
                        6'd40: gamma6 = 6'd33;
                        6'd41: gamma6 = 6'd35;
                        6'd42: gamma6 = 6'd36;
                        6'd43: gamma6 = 6'd37;
                        6'd44: gamma6 = 6'd38;
                        6'd45: gamma6 = 6'd39;
                        6'd46: gamma6 = 6'd41;
                        6'd47: gamma6 = 6'd42;
                        6'd48: gamma6 = 6'd43;
                        6'd49: gamma6 = 6'd44;
                        6'd50: gamma6 = 6'd46;
                        6'd51: gamma6 = 6'd47;
                        6'd52: gamma6 = 6'd48;
                        6'd53: gamma6 = 6'd49;
                        6'd54: gamma6 = 6'd51;
                        6'd55: gamma6 = 6'd52;
                        6'd56: gamma6 = 6'd53;
                        6'd57: gamma6 = 6'd55;
                        6'd58: gamma6 = 6'd56;
                        6'd59: gamma6 = 6'd57;
                        6'd60: gamma6 = 6'd59;
                        6'd61: gamma6 = 6'd60;
                        6'd62: gamma6 = 6'd62;
                        6'd63: gamma6 = 6'd63;
                        default: gamma6 = code;
                    endcase
                end
                default: gamma6 = code;
            endcase
        end
    endfunction

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            pending_mode <= RESET_MODE;
            applied_mode <= RESET_MODE;
            gamma_id <= RESET_GAMMA;
            auto_gamma <= 2'd0;
            confirm_target <= 2'd0;
            confirm_count <= 32'd0;
            luma_sum <= 32'd0;
            pixel_count <= 32'd0;
            nonzero_count <= 32'd0;
            out_hs <= 1'b0;
            out_vs <= 1'b0;
            out_de <= 1'b0;
            out_rgb565 <= 16'd0;
            status_valid <= 1'b0;
            status_data <= {RESET_MODE, RESET_GAMMA};
        end else begin
            vs_d <= vs;
            status_valid <= 1'b0;
            if (config_valid) pending_mode <= config_mode;
            out_hs <= hs;
            out_vs <= vs;
            out_de <= de;
            // Identity is explicitly bit-exact, including blanking pixels.
            if (gamma_id == 2'd0) out_rgb565 <= rgb565;
            else out_rgb565 <= {gamma5(rgb565[15:11], gamma_id),
                                gamma6(rgb565[10:5], gamma_id),
                                gamma5(rgb565[4:0], gamma_id)};
            if (frame_boundary) begin
                applied_mode <= next_mode;
                gamma_id <= next_gamma;
                auto_gamma <= next_auto;
                confirm_target <= next_confirm_target;
                confirm_count <= next_confirm_count;
                luma_sum <= 32'd0;
                pixel_count <= 32'd0;
                nonzero_count <= 32'd0;
                status_valid <= 1'b1;
                status_data <= {next_mode, next_gamma};
            end else if (de) begin
                luma_sum <= luma_sum + {24'd0, input_luma};
                // Saturation prevents excessively long malformed frames from
                // wrapping back into an apparently exact frame pixel count.
                if (pixel_count != 32'hffffffff) pixel_count <= pixel_count + 32'd1;
                if (rgb565 != 16'd0 && nonzero_count != 32'hffffffff)
                    nonzero_count <= nonzero_count + 32'd1;
            end
        end
    end
endmodule
