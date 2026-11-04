`timescale 1ns/1ps
// Receive-only configuration protocol, 115200 8N1 by default. NO ACK is sent.
// Packet: A5, address, data[7:0], data[15:8], data[23:16], data[31:24], CRC_hi, CRC_lo.
// CRC16/CCITT-FALSE: polynomial 1021, initial FFFF, no reflection/xorout,
// calculated over all first six bytes (including A5).
// Addresses 00..0F write one register. FF with data=434F4D54 requests commit.
// E0/E1 belong to the independent SCCB adapter and are ignored here.
// E2 with data=0..3 updates the independent enhancement mode immediately.
// Other valid-CRC commands are packet errors. A write is held until cfg_ready;
// one pending write is buffered and additional writes are rejected while full.
// Commit is rejected immediately unless commit_ready and no pending write.
// The host cannot infer FPGA acceptance from a successful serial-port write.
module uart_config_rx_advanced #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD = 115200,
    parameter integer REGISTER_COUNT = 16,
    parameter integer PACKET_TIMEOUT_CYCLES = CLK_HZ/50
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    output reg  [4:0]  core_cfg_addr,
    output reg  [31:0] core_cfg_data,
    output reg         core_cfg_valid,
    input  wire        core_cfg_ready,
    output reg         core_cfg_commit,
    input  wire        core_cfg_commit_ready,
    output reg  [31:0] crc_error_count,
    output reg  [31:0] framing_error_count,
    output reg  [31:0] packet_error_count,
    output reg  [31:0] rejected_count,
    output reg  [31:0] write_count,
    output reg  [31:0] commit_count,
    output reg         enhance_valid,
    output reg  [1:0]  enhance_mode
);
    wire [7:0] rx_byte;
    wire rx_valid, rx_bad_stop;
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) receiver (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx), .data(rx_byte),
        .valid(rx_valid), .framing_error(rx_bad_stop), .busy()
    );

    reg [7:0] packet [0:7];
    reg [3:0] byte_count;
    reg verifying;
    reg [2:0] crc_index;
    reg [15:0] crc;
    reg [31:0] idle_cycles;
    integer j, resync_index;
    wire [31:0] packet_data = {packet[5],packet[4],packet[3],packet[2]};

    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] work;
        integer bit_no;
        begin
            work = previous ^ {value,8'b0};
            for (bit_no = 0; bit_no < 8; bit_no = bit_no+1)
                if (work[15]) work = (work << 1) ^ 16'h1021;
                else work = work << 1;
            crc_byte = work;
        end
    endfunction

    function [31:0] increment_saturated;
        input [31:0] value;
        begin increment_saturated = (value == 32'hffffffff) ? value : value+32'd1; end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_cfg_addr <= 0; core_cfg_data <= 0; core_cfg_valid <= 0; core_cfg_commit <= 0;
            crc_error_count <= 0; framing_error_count <= 0; packet_error_count <= 0;
            rejected_count <= 0; write_count <= 0; commit_count <= 0;
            enhance_valid <= 0; enhance_mode <= 0;
            byte_count <= 0; verifying <= 0; crc_index <= 0; crc <= 16'hffff; idle_cycles <= 0;
            for (j = 0; j < 8; j = j+1) packet[j] <= 0;
        end else begin
            core_cfg_commit <= 0;
            enhance_valid <= 0;
            if (core_cfg_valid && core_cfg_ready) begin
                core_cfg_valid <= 0;
                write_count <= increment_saturated(write_count);
            end
            if (core_cfg_commit) begin
                if (core_cfg_commit_ready) commit_count <= increment_saturated(commit_count);
                else rejected_count <= increment_saturated(rejected_count);
            end

            if (rx_bad_stop) begin
                framing_error_count <= increment_saturated(framing_error_count);
                byte_count <= 0; verifying <= 0; idle_cycles <= 0;
            end else if (verifying) begin
                // Six byte steps avoid a long combinational 48-bit CRC chain.
                // UART byte intervals are >=40 clocks, so no byte can arrive here.
                if (crc_index < 6) begin
                    crc <= crc_byte(crc, packet[crc_index]);
                    crc_index <= crc_index+1'b1;
                end else begin
                    verifying <= 0; idle_cycles <= 0;
                    if (crc == {packet[6],packet[7]}) begin
                        byte_count <= 0;
                        if (packet[1] < REGISTER_COUNT && packet[1] < 8'd32) begin
                            if (!core_cfg_valid || core_cfg_ready) begin
                                core_cfg_addr <= packet[1][4:0];
                                core_cfg_data <= packet_data; core_cfg_valid <= 1;
                            end else rejected_count <= increment_saturated(rejected_count);
                        end else if ((packet[1] == 8'hff) && (packet_data == 32'h434f4d54)) begin
                            if (core_cfg_commit_ready && !core_cfg_valid) core_cfg_commit <= 1;
                            else rejected_count <= increment_saturated(rejected_count);
                        end else if (packet[1] == 8'he0 || packet[1] == 8'he1) begin
                            // Independent sensor-register diagnostic namespace.
                        end else if (packet[1] == 8'he2) begin
                            // Validate all 32 data bits before updating the mode.
                            if (packet_data[31:2] == 30'd0) begin
                                enhance_mode <= packet_data[1:0];
                                enhance_valid <= 1;
                            end else packet_error_count <= increment_saturated(packet_error_count);
                        end else packet_error_count <= increment_saturated(packet_error_count);
                    end else begin
                        crc_error_count <= increment_saturated(crc_error_count);
                        // Preserve the earliest later A5 suffix. This recovers a valid
                        // packet following a truncated packet even without an idle gap.
                        resync_index = 8;
                        for (j = 1; j < 8; j = j+1)
                            if ((packet[j] == 8'ha5) && (resync_index == 8)) resync_index = j;
                        for (j = 0; j < 7; j = j+1)
                            if (j < 8-resync_index) packet[j] <= packet[j+resync_index];
                        byte_count <= 8-resync_index;
                    end
                end
            end else if (rx_valid) begin
                idle_cycles <= 0;
                if (byte_count == 0) begin
                    if (rx_byte == 8'ha5) begin packet[0] <= rx_byte; byte_count <= 1; end
                end else begin
                    packet[byte_count] <= rx_byte;
                    if (byte_count == 7) begin
                        byte_count <= 8; verifying <= 1; crc_index <= 0; crc <= 16'hffff;
                    end else byte_count <= byte_count+1'b1;
                end
            end else if (byte_count != 0) begin
                if (idle_cycles >= PACKET_TIMEOUT_CYCLES-1) begin
                    byte_count <= 0; idle_cycles <= 0;
                    packet_error_count <= increment_saturated(packet_error_count);
                end else idle_cycles <= idle_cycles+32'd1;
            end else idle_cycles <= 0;
        end
    end
endmodule
