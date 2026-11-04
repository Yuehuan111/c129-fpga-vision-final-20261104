`timescale 1ns/1ps
// Full-size BGGR bilinear demosaic. Two synchronous 8-bit line memories;
// one input/output sample per clock. The first/last rows and columns use
// clamped sample coordinates. A synthetic final row drains in vertical blank.
// The validated RAW frame event is held until all output pixels have drained.
// No exposure/white-balance/gamma operation is implied by this module.
module bayer_bggr_to_rgb565 #(
    parameter integer IMG_WIDTH=1920, IMG_HEIGHT=1080
)(
    input wire pclk,rst_n,
    input wire in_valid,
    input wire [7:0] in_raw8,
    input wire [15:0] in_x,in_y,
    input wire in_sof,in_eol,in_eof,
    input wire in_frame_valid_done,in_frame_drop,
    output reg pix_valid,
    output reg [15:0] rgb565,x,y,
    output reg sof,eol,eof,frame_valid_done,frame_drop,frame_error
);
    // Even/odd sensor rows occupy separate banks. Each RAM has exactly one
    // synchronous read and one write port: no line1 -> line2 asynchronous copy.
    // Read-before-write returns row y-2 while this pixel overwrites that bank.
    localparam RAM_ADDR_BITS=(IMG_WIDTH<2)?1:$clog2(IMG_WIDTH);
    reg [7:0] line_even[0:IMG_WIDTH-1];
    reg [7:0] line_odd[0:IMG_WIDTH-1];
    reg [7:0] even_q,odd_q;
    reg st_valid;
    reg [15:0] st_x,st_y;
    reg [7:0] st_raw;
    reg flushing;
    reg [15:0] flush_x;
    reg [7:0] t0,t1,m0,m1,b0,b1;
    reg right_pending;
    reg [15:0] right_y;
    reg output_complete,validation_pending,local_bad;
    wire take_real=rst_n && !in_frame_drop && in_valid && !flushing &&
                   in_x<IMG_WIDTH && in_y<IMG_HEIGHT;
    wire take_flush=rst_n && !in_frame_drop && !in_valid && flushing;
    wire [RAM_ADDR_BITS-1:0] ram_x=take_real?in_x[RAM_ADDR_BITS-1:0]:flush_x[RAM_ADDR_BITS-1:0];
    wire [7:0] st_t=st_y[0]?odd_q:even_q;
    wire [7:0] st_m=st_y[0]?even_q:odd_q;
    wire [7:0] st_b=(st_y==IMG_HEIGHT)?st_m:st_raw;
    wire [7:0] top_now=(st_y==1) ? st_m : st_t;
    wire [7:0] left_t=(st_x==1) ? t1 : t0;
    wire [7:0] left_m=(st_x==1) ? m1 : m0;
    wire [7:0] left_b=(st_x==1) ? b1 : b0;

    // Keep memory and its output registers out of the asynchronous-reset
    // process. st_valid masks stale RAM data until the first line is written.
    always @(posedge pclk) begin
        if(take_real || take_flush) begin
            even_q<=line_even[ram_x];
            odd_q<=line_odd[ram_x];
        end
        if(take_real && !in_y[0]) line_even[ram_x]<=in_raw8;
        if(take_real && in_y[0]) line_odd[ram_x]<=in_raw8;
    end

    function [15:0] interpolate;
        input [7:0] tl,tc,tr,ml,mc,mr,bl,bc,br;
        input px,py;
        reg [9:0] rr,gg,bb;
        begin
            rr=0;gg=0;bb=0;
            case({py,px})
            2'b00: begin // B
                bb={2'b0,mc};
                gg=({2'b0,tc}+{2'b0,ml}+{2'b0,mr}+{2'b0,bc}+10'd2)>>2;
                rr=({2'b0,tl}+{2'b0,tr}+{2'b0,bl}+{2'b0,br}+10'd2)>>2;
            end
            2'b01: begin // G on the B row
                gg={2'b0,mc};
                bb=({2'b0,ml}+{2'b0,mr}+10'd1)>>1;
                rr=({2'b0,tc}+{2'b0,bc}+10'd1)>>1;
            end
            2'b10: begin // G on the R row
                gg={2'b0,mc};
                rr=({2'b0,ml}+{2'b0,mr}+10'd1)>>1;
                bb=({2'b0,tc}+{2'b0,bc}+10'd1)>>1;
            end
            2'b11: begin // R
                rr={2'b0,mc};
                gg=({2'b0,tc}+{2'b0,ml}+{2'b0,mr}+{2'b0,bc}+10'd2)>>2;
                bb=({2'b0,tl}+{2'b0,tr}+{2'b0,bl}+{2'b0,br}+10'd2)>>2;
            end
            endcase
            interpolate={rr[7:3],gg[7:2],bb[7:3]};
        end
    endfunction

    always @(posedge pclk or negedge rst_n) begin
        if(!rst_n) begin
            st_valid<=0;st_x<=0;st_y<=0;st_raw<=0;
            flushing<=0;flush_x<=0;right_pending<=0;right_y<=0;
            t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;
            pix_valid<=0;rgb565<=0;x<=0;y<=0;sof<=0;eol<=0;eof<=0;
            frame_valid_done<=0;frame_drop<=0;frame_error<=0;
            output_complete<=0;validation_pending<=0;local_bad<=0;
        end else begin
            st_valid<=0;pix_valid<=0;sof<=0;eol<=0;eof<=0;
            frame_valid_done<=0;frame_drop<=0;right_pending<=0;
            if(in_frame_valid_done) validation_pending<=1;
            if(output_complete && (validation_pending || in_frame_valid_done)) begin
                if(!local_bad) frame_valid_done<=1;
                else frame_drop<=1;
                output_complete<=0;validation_pending<=0;
            end
            if(in_valid && in_sof && !flushing) local_bad<=0;

            // Metadata and the synchronous RAM read have the same latency.
            if(in_valid) begin
                if(flushing || in_x>=IMG_WIDTH || in_y>=IMG_HEIGHT) begin
                    frame_error<=1;frame_drop<=1;local_bad<=1;
                end else begin
                    st_valid<=1;st_x<=in_x;st_y<=in_y;
                    st_raw<=in_raw8;
                    if(in_eof) begin flushing<=1;flush_x<=0;end
                end
            end else if(flushing) begin
                st_valid<=1;st_x<=flush_x;st_y<=IMG_HEIGHT;
                if(flush_x==IMG_WIDTH-1) flushing<=0;
                else flush_x<=flush_x+1'b1;
            end

            // Replicate the last column after its right neighbour is known.
            // This can coincide with x=0 of the following row, which emits no
            // pixel and only primes the new window.
            if(right_pending) begin
                pix_valid<=1;x<=IMG_WIDTH-1;y<=right_y;
                sof<=(IMG_WIDTH==1 && right_y==0);eol<=1;
                eof<=(right_y==IMG_HEIGHT-1);
                rgb565<=interpolate(IMG_WIDTH==1?t1:t0,t1,t1,
                                    IMG_WIDTH==1?m1:m0,m1,m1,
                                    IMG_WIDTH==1?b1:b0,b1,b1,
                                    (IMG_WIDTH-1)%2,right_y[0]);
                if(right_y==IMG_HEIGHT-1) output_complete<=1;
            end
            if(st_valid) begin
                t0<=t1;t1<=top_now;m0<=m1;m1<=st_m;b0<=b1;b1<=st_b;
                if(st_y!=0) begin
                    if(st_x!=0) begin
                        if(right_pending) begin frame_error<=1;frame_drop<=1;local_bad<=1;end
                        pix_valid<=1;x<=st_x-1'b1;y<=st_y-1'b1;
                        sof<=(st_x==1 && st_y==1);eol<=0;eof<=0;
                        rgb565<=interpolate(left_t,t1,top_now,left_m,m1,st_m,
                                            left_b,b1,st_b,!st_x[0],!st_y[0]);
                    end
                    if(st_x==IMG_WIDTH-1) begin right_pending<=1;right_y<=st_y-1'b1;end
                end
            end
            // A malformed frame is never published, including errors reported
            // after nominal eof. The next well-formed frame fully recovers.
            if(in_frame_drop) begin
                frame_drop<=1;frame_error<=1;st_valid<=0;flushing<=0;
                right_pending<=0;output_complete<=0;validation_pending<=0;
                frame_valid_done<=0;local_bad<=0;
            end
        end
    end
endmodule
