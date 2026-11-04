`timescale 1ns/1ps
// Independent receive-only SCCB command decoder. Existing parameter addresses
// 00..0F and commit FF never issue a sensor transaction. Reuses uart_rx.v.
// A5 E0 reg_lo reg_hi 00 00 CRC_hi CRC_lo: read one sensor register.
// A5 E1 reg_lo reg_hi data 00 CRC_hi CRC_lo: write one sensor register.
// CRC16/CCITT-FALSE over the first six bytes, identical to uart_config_rx.
// Busy/not-enabled commands are rejected, never queued for a surprise later write.
module uart_sccb_adapter #(
    parameter integer CLK_HZ=100000000,
    parameter integer BAUD=115200,
    parameter integer PACKET_TIMEOUT_CYCLES=CLK_HZ/50
) (
    input wire clk, rst_n, uart_rx, boot_ready,
    output reg cmd_valid,
    input wire cmd_ready,
    output reg cmd_write,
    output reg [15:0] cmd_register,
    output reg [7:0] cmd_data,
    output reg [31:0] accepted_count, rejected_count, crc_error_count,
    output reg [31:0] framing_error_count, packet_error_count, ignored_count,
    // Last rejection: 1=boot ownership unavailable; 2=busy; 3=reserved bits.
    output reg [3:0] last_reject_reason
);
    wire [7:0] rx_byte;
    wire rx_valid, rx_bad_stop;
    uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver (
        .clk(clk),.rst_n(rst_n),.rx(uart_rx),.data(rx_byte),
        .valid(rx_valid),.framing_error(rx_bad_stop),.busy()
    );
    reg [7:0] packet[0:7];
    reg [3:0] byte_count;
    reg verifying;
    reg [2:0] crc_index;
    reg [15:0] crc;
    reg [31:0] idle_cycles;
    integer j, resync_index;
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] work;
        integer b;
        begin
            work=previous^{value,8'b0};
            for (b=0;b<8;b=b+1)
                work=work[15] ? (work<<1)^16'h1021 : work<<1;
            crc_byte=work;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_valid<=0; cmd_write<=0; cmd_register<=0; cmd_data<=0;
            accepted_count<=0; rejected_count<=0; crc_error_count<=0;
            framing_error_count<=0; packet_error_count<=0; ignored_count<=0;
            last_reject_reason<=0; byte_count<=0; verifying<=0;
            crc_index<=0; crc<=16'hffff; idle_cycles<=0;
            for (j=0;j<8;j=j+1) packet[j]<=0;
        end else begin
            if (cmd_valid) begin
                if (cmd_ready) begin cmd_valid<=0; accepted_count<=accepted_count+1'b1; end
                else if (!boot_ready) begin
                    cmd_valid<=0; rejected_count<=rejected_count+1'b1; last_reject_reason<=1;
                end
            end
            if (rx_bad_stop) begin
                framing_error_count<=framing_error_count+1'b1;
                byte_count<=0; verifying<=0; idle_cycles<=0;
            end else if (verifying) begin
                if (crc_index<6) begin
                    crc<=crc_byte(crc,packet[crc_index]); crc_index<=crc_index+1'b1;
                end else begin
                    verifying<=0; idle_cycles<=0;
                    if (crc=={packet[6],packet[7]}) begin
                        byte_count<=0;
                        if (packet[1]==8'he0 || packet[1]==8'he1) begin
                            if (packet[5]!=0 || (packet[1]==8'he0 && packet[4]!=0)) begin
                                packet_error_count<=packet_error_count+1'b1;
                                rejected_count<=rejected_count+1'b1; last_reject_reason<=3;
                            end else if (!boot_ready) begin
                                rejected_count<=rejected_count+1'b1; last_reject_reason<=1;
                            end else if (!cmd_ready || cmd_valid) begin
                                rejected_count<=rejected_count+1'b1; last_reject_reason<=2;
                            end else begin
                                cmd_write<=packet[1]==8'he1;
                                cmd_register<={packet[3],packet[2]}; cmd_data<=packet[4];
                                cmd_valid<=1;
                            end
                        end else ignored_count<=ignored_count+1'b1;
                    end else begin
                        crc_error_count<=crc_error_count+1'b1;
                        resync_index=8;
                        for (j=1;j<8;j=j+1)
                            if (packet[j]==8'ha5 && resync_index==8) resync_index=j;
                        for (j=0;j<7;j=j+1)
                            if (j<8-resync_index) packet[j]<=packet[j+resync_index];
                        byte_count<=8-resync_index;
                    end
                end
            end else if (rx_valid) begin
                idle_cycles<=0;
                if (byte_count==0) begin
                    if (rx_byte==8'ha5) begin packet[0]<=rx_byte; byte_count<=1; end
                end else begin
                    packet[byte_count]<=rx_byte;
                    if (byte_count==7) begin
                        byte_count<=8; verifying<=1; crc_index<=0; crc<=16'hffff;
                    end else byte_count<=byte_count+1'b1;
                end
            end else if (byte_count!=0) begin
                if (idle_cycles>=PACKET_TIMEOUT_CYCLES-1) begin
                    byte_count<=0; idle_cycles<=0; packet_error_count<=packet_error_count+1'b1;
                end else idle_cycles<=idle_cycles+1'b1;
            end else idle_cycles<=0;
        end
    end
endmodule
