`timescale 1ns/1ps
// DVP RGB565 receiver. All signals and outputs are synchronous to pclk.
// A pixel consists of two href-qualified bytes, high byte first. There is no
// backpressure: the camera cannot be stalled. Sample the byte bus on pclk rising.
//
// sof/eol/eof qualify pix_valid and identify the nominal raster boundaries.
// A later extra byte/line can still invalidate an already emitted raster.
// Commit a buffered frame ONLY on frame_valid_done, at the next active VSYNC
// edge; frame_drop at that boundary invalidates the preceding partial frame.
// frame_error is sticky until reset. drop_count saturates and counts bad frames,
// not individual bad lines. Data before the first active VSYNC is ignored.
module dvp_rgb565_rx #(
    parameter integer IMG_WIDTH = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer VSYNC_ACTIVE_LEVEL = 1,
    parameter integer SWAP_RB = 1
) (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        vsync,
    input  wire        href,
    input  wire [7:0]  data,
    output reg         pix_valid,
    output reg  [15:0] rgb565,
    output reg  [15:0] x,
    output reg  [15:0] y,
    output reg         sof,
    output reg         eol,
    output reg         eof,
    output reg         frame_valid_done,
    output reg         frame_drop,
    output reg         frame_error,
    output reg  [31:0] drop_count
);

    wire sync_active = (VSYNC_ACTIVE_LEVEL != 0) ? vsync : !vsync;
    reg sync_was_active;
    reg sync_seen;
    reg line_active;
    reg byte_pending;
    reg [7:0] high_byte;
    reg [31:0] line_pixels;
    reg [31:0] lines_seen;
    reg frame_bad;
    wire [15:0] assembled_pixel = {high_byte, data};
    wire [15:0] converted_pixel = (SWAP_RB != 0) ?
        {assembled_pixel[4:0], assembled_pixel[10:5], assembled_pixel[15:11]} :
        assembled_pixel;

    // Accept href falling on the same sampled edge as active VSYNC, provided
    // that it closes a complete final line. Active VSYNC with href high aborts.
    wire simultaneous_final_line_end = line_active && !href && !byte_pending &&
        (line_pixels == IMG_WIDTH) && (lines_seen == IMG_HEIGHT-1);
    wire preceding_frame_complete = !frame_bad && !href &&
        ((!line_active && (lines_seen == IMG_HEIGHT)) || simultaneous_final_line_end);

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            pix_valid <= 0;
            rgb565 <= 0;
            x <= 0;
            y <= 0;
            sof <= 0;
            eol <= 0;
            eof <= 0;
            frame_valid_done <= 0;
            frame_drop <= 0;
            frame_error <= 0;
            drop_count <= 0;
            sync_was_active <= 0;
            sync_seen <= 0;
            line_active <= 0;
            byte_pending <= 0;
            high_byte <= 0;
            line_pixels <= 0;
            lines_seen <= 0;
            frame_bad <= 0;
        end else begin
            pix_valid <= 0;
            sof <= 0;
            eol <= 0;
            eof <= 0;
            frame_valid_done <= 0;
            frame_drop <= 0;
            sync_was_active <= sync_active;

            if (sync_active) begin
                if (!sync_was_active) begin
                    if (sync_seen) begin
                        if (preceding_frame_complete)
                            frame_valid_done <= 1;
                        else begin
                            frame_drop <= 1;
                            frame_error <= 1;
                            if (drop_count != 32'hffffffff)
                                drop_count <= drop_count + 32'd1;
                        end
                    end
                    sync_seen <= 1;
                end
                // No data is collected during the whole active VSYNC interval.
                line_active <= 0;
                byte_pending <= 0;
                high_byte <= 0;
                line_pixels <= 0;
                lines_seen <= 0;
                frame_bad <= 0;
            end else if (sync_seen) begin
                if (href) begin
                    if (!line_active) begin
                        line_active <= 1;
                        byte_pending <= 1;
                        high_byte <= data;
                        line_pixels <= 0;
                        if (lines_seen >= IMG_HEIGHT) begin
                            frame_bad <= 1;
                            frame_error <= 1;
                        end
                    end else if (!byte_pending) begin
                        byte_pending <= 1;
                        high_byte <= data;
                        if (line_pixels >= IMG_WIDTH) begin
                            frame_bad <= 1;
                            frame_error <= 1;
                        end
                    end else begin
                        byte_pending <= 0;
                        if ((line_pixels < IMG_WIDTH) && (lines_seen < IMG_HEIGHT) && !frame_bad) begin
                            pix_valid <= 1;
                            rgb565 <= converted_pixel;
                            x <= line_pixels[15:0];
                            y <= lines_seen[15:0];
                            sof <= (line_pixels == 0) && (lines_seen == 0);
                            eol <= (line_pixels == IMG_WIDTH-1);
                            eof <= (line_pixels == IMG_WIDTH-1) && (lines_seen == IMG_HEIGHT-1);
                        end
                        if (line_pixels < IMG_WIDTH+1)
                            line_pixels <= line_pixels + 32'd1;
                        if (line_pixels >= IMG_WIDTH) begin
                            frame_bad <= 1;
                            frame_error <= 1;
                        end
                    end
                end else if (line_active) begin
                    line_active <= 0;
                    byte_pending <= 0; // Never join bytes from different lines.
                    if (byte_pending || (line_pixels != IMG_WIDTH) || (lines_seen >= IMG_HEIGHT)) begin
                        frame_bad <= 1;
                        frame_error <= 1;
                    end
                    if (lines_seen < IMG_HEIGHT+1)
                        lines_seen <= lines_seen + 32'd1;
                end
            end
        end
    end
endmodule
