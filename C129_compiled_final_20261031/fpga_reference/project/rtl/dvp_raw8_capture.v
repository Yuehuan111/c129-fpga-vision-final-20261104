`timescale 1ns/1ps
// One href-qualified byte is one RAW8 Bayer sample on D[9:2].
// Nominal eof does not validate a frame: validation waits for the next active
// VSYNC edge, so extra samples/lines cannot publish a corrupt frame.
module dvp_raw8_capture #(
    parameter integer IMG_WIDTH=1920, IMG_HEIGHT=1080,
    parameter integer VSYNC_ACTIVE_LEVEL=1
)(
    input wire pclk,rst_n,vsync,href,
    input wire [7:0] data,
    output reg pix_valid,
    output reg [7:0] raw8,
    output reg [15:0] x,y,
    output reg sof,eol,eof,frame_valid_done,frame_drop,frame_error,
    output reg [31:0] drop_count
);
    wire sync_active=VSYNC_ACTIVE_LEVEL ? vsync : !vsync;
    reg sync_was_active,sync_seen,line_active,frame_bad;
    reg [31:0] line_pixels,lines_seen;
    wire closes_last_line=line_active && !href &&
        line_pixels==IMG_WIDTH && lines_seen==IMG_HEIGHT-1;
    wire frame_complete=!frame_bad && !href &&
        ((!line_active && lines_seen==IMG_HEIGHT) || closes_last_line);
    always @(posedge pclk or negedge rst_n) begin
        if(!rst_n) begin
            pix_valid<=0;raw8<=0;x<=0;y<=0;sof<=0;eol<=0;eof<=0;
            frame_valid_done<=0;frame_drop<=0;frame_error<=0;drop_count<=0;
            sync_was_active<=0;sync_seen<=0;line_active<=0;frame_bad<=0;
            line_pixels<=0;lines_seen<=0;
        end else begin
            pix_valid<=0;sof<=0;eol<=0;eof<=0;
            frame_valid_done<=0;frame_drop<=0;sync_was_active<=sync_active;
            if(sync_active) begin
                if(!sync_was_active) begin
                    if(sync_seen) begin
                        if(frame_complete) frame_valid_done<=1;
                        else begin
                            frame_drop<=1;frame_error<=1;
                            if(drop_count!=32'hffffffff) drop_count<=drop_count+1'b1;
                        end
                    end
                    sync_seen<=1;
                end
                line_active<=0;frame_bad<=0;line_pixels<=0;lines_seen<=0;
            end else if(sync_seen) begin
                if(href) begin
                    line_active<=1;
                    if(!line_active) line_pixels<=1;
                    else if(line_pixels<IMG_WIDTH+1) line_pixels<=line_pixels+1'b1;
                    if((!line_active || line_pixels<IMG_WIDTH) &&
                       lines_seen<IMG_HEIGHT && !frame_bad) begin
                        pix_valid<=1;raw8<=data;
                        x<=line_active ? line_pixels[15:0] : 16'd0;
                        y<=lines_seen[15:0];
                        sof<=(!line_active || line_pixels==0) && lines_seen==0;
                        eol<=(!line_active ? IMG_WIDTH==1 : line_pixels==IMG_WIDTH-1);
                        eof<=(!line_active ? IMG_WIDTH==1 : line_pixels==IMG_WIDTH-1) && lines_seen==IMG_HEIGHT-1;
                    end
                    if(lines_seen>=IMG_HEIGHT || (line_active && line_pixels>=IMG_WIDTH)) begin
                        frame_bad<=1;frame_error<=1;
                    end
                end else if(line_active) begin
                    line_active<=0;
                    if(line_pixels!=IMG_WIDTH || lines_seen>=IMG_HEIGHT) begin
                        frame_bad<=1;frame_error<=1;
                    end
                    if(lines_seen<IMG_HEIGHT+1) lines_seen<=lines_seen+1'b1;
                end
            end
        end
    end
endmodule
