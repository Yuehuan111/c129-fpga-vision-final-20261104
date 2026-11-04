`timescale 1ns/1ps
// Configuration receiver, 115200 8N1. C125 POS1 events are sent by the TX owner.
// Packet: A5, address, data[7:0], data[15:8], data[23:16], data[31:24], CRC_hi, CRC_lo.
// CRC16/CCITT-FALSE: polynomial 1021, initial FFFF, no reflection/xorout,
// calculated over all first six bytes (including A5).
// Addresses 00..11 write the original 18 words; 20..28 write row-major
// signed Q16.16 inverse-projective H[0..8]. FF / 434F4D54 requests commit.
// FE / 434F4D50 starts a new transaction (core addr3F/data434F4D50).
// FD / 504Fvvvv starts an H-only transaction based on published version vvvv.
// FC / 5649vvvv starts the source-observed visual H-only transaction.
// EC / 0,1,2 selects corrected, raw, or corrected+source observation.
// The key arbiter validates the base version and expands saved optical words.
// CRC/framing/timeout failures and malformed FE abort (core addr3F/data0).
// The controller requires BEGIN then all 18+9 words; commit_ready owns this check.
// E0/E1 belong to the independent SCCB adapter and are ignored here.
// E2 with data=0..4 updates the independent enhancement mode immediately.
// C129 E4/data0 or1 requests bounded detail off/on, applied at frame boundary.
// Other valid-CRC commands are packet errors. A write is held until cfg_ready;
// one pending write is buffered and additional writes are rejected while full.
// Commit is rejected immediately unless commit_ready and no pending write.
// The host cannot infer FPGA acceptance from a successful serial-port write.
module uart_config_rx_complete #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD = 115200,
    parameter integer REGISTER_COUNT = 18,
    parameter integer PACKET_TIMEOUT_CYCLES = CLK_HZ/50
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    output reg  [5:0]  core_cfg_addr,
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
    output reg  [2:0]  enhance_mode,
    output reg         detail_valid,
    output reg         detail_enable,
    output reg         aux_valid,
    output reg  [5:0]  aux_addr,
    output reg  [31:0] aux_data,
    input  wire        aux_ready
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
    // Packet fields are stable throughout CRC verification. Predecode them
    // on its first cycle; dynamic ready/pending decisions stay on the last.
    (* syn_preserve=1 *) reg decoded_write, decoded_begin, decoded_commit;
    (* syn_preserve=1 *) reg decoded_sccb, decoded_enhance, decoded_pose;
    (* syn_preserve=1 *) reg decoded_visual;
    (* syn_preserve=1 *) reg decoded_detail;
    (* syn_preserve=1 *) reg decoded_aux;
    (* syn_preserve=1 *) reg decoded_begin_magic, decoded_commit_magic, decoded_mode_range;
    reg [2:0] crc_index;
    reg [15:0] crc;
    reg [31:0] idle_cycles;
    // Keep packet/CRC decoding off the rejected counter's increment enable.
    // Each rejection is counted once on the following clock; reset cancels it.
    (* syn_preserve=1 *) reg reject_pending;
    // Cached carry/saturation predicates describe the CURRENT commit_count.
    // Updating them from the pre-increment ...FE values also supports commits
    // on consecutive clocks without putting a 32-bit carry chain on high CEs.
    (* syn_preserve=1 *) reg commit_carry8, commit_carry16, commit_carry24, commit_saturated;
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
            reject_pending <= 0;
            commit_carry8 <= 0; commit_carry16 <= 0; commit_carry24 <= 0; commit_saturated <= 0;
            enhance_valid <= 0; enhance_mode <= 0;
            detail_valid <= 0; detail_enable <= 0; decoded_detail <= 0;
            aux_valid <= 0; aux_addr <= 0; aux_data <= 0; decoded_aux <= 0;
            byte_count <= 0; verifying <= 0; crc_index <= 0; crc <= 16'hffff; idle_cycles <= 0;
            decoded_write <= 0; decoded_begin <= 0; decoded_commit <= 0;
            decoded_sccb <= 0; decoded_enhance <= 0; decoded_pose <= 0;
            decoded_visual <= 0;
            decoded_begin_magic <= 0; decoded_commit_magic <= 0; decoded_mode_range <= 0;
            for (j = 0; j < 8; j = j+1) packet[j] <= 0;
        end else begin
            reject_pending <= 0;
            if (reject_pending) rejected_count <= increment_saturated(rejected_count);
            core_cfg_commit <= 0;
            enhance_valid <= 0;
            detail_valid <= 0;
            if (aux_valid && aux_ready) aux_valid <= 0;
            if (core_cfg_valid && core_cfg_ready) begin
                core_cfg_valid <= 0;
                write_count <= increment_saturated(write_count);
            end
            if (core_cfg_commit) begin
                if (core_cfg_commit_ready) begin
                    if (!commit_saturated) begin
                        commit_count[7:0] <= commit_count[7:0]+8'd1;
                        if (commit_carry8) commit_count[15:8] <= commit_count[15:8]+8'd1;
                        if (commit_carry16) commit_count[23:16] <= commit_count[23:16]+8'd1;
                        if (commit_carry24) commit_count[31:24] <= commit_count[31:24]+8'd1;
                        commit_carry8 <= commit_count[7:0] == 8'hfe;
                        commit_carry16 <= commit_count[15:0] == 16'hfffe;
                        commit_carry24 <= commit_count[23:0] == 24'hfffffe;
                        commit_saturated <= commit_count == 32'hfffffffe;
                    end
                end else reject_pending <= 1;
            end

            if (rx_bad_stop) begin
                framing_error_count <= increment_saturated(framing_error_count);
                byte_count <= 0; verifying <= 0; idle_cycles <= 0;
                core_cfg_addr <= 6'h3f; core_cfg_data <= 0; core_cfg_valid <= 1;
                aux_addr <= 6'h3a; aux_data <= 0; aux_valid <= 1;
            end else if (verifying) begin
                // Six byte steps avoid a long combinational 48-bit CRC chain.
                // UART byte intervals are >=40 clocks, so no byte can arrive here.
                if (crc_index == 0) begin
                    decoded_write <= ((packet[1] < REGISTER_COUNT) && (packet[1] < 8'd18)) ||
                                     ((packet[1] >= 8'h20) && (packet[1] <= 8'h28));
                    decoded_begin <= packet[1] == 8'hfe;
                    decoded_pose <= packet[1] == 8'hfd || packet[1] == 8'hfc;
                    decoded_visual <= packet[1] == 8'hfc;
                    decoded_commit <= packet[1] == 8'hff;
                    decoded_sccb <= packet[1] == 8'he0 || packet[1] == 8'he1;
                    decoded_enhance <= packet[1] == 8'he2;
                    decoded_detail <= packet[1] == 8'he4;
                    decoded_aux <= (packet[1] >= 8'h40 && packet[1] <= 8'h63) ||
                                   (packet[1] >= 8'he7 && packet[1] <= 8'hec);
                    decoded_begin_magic <= packet_data == 32'h434f4d50;
                    decoded_commit_magic <= packet_data == 32'h434f4d54;
                    decoded_mode_range <= packet_data <= 32'd4;
                end
                if (crc_index < 6) begin
                    crc <= crc_byte(crc, packet[crc_index]);
                    crc_index <= crc_index+1'b1;
                end else begin
                    verifying <= 0; idle_cycles <= 0;
                    if (crc == {packet[6],packet[7]}) begin
                        byte_count <= 0;
                        if (decoded_write) begin
                            if (!core_cfg_valid || core_cfg_ready) begin
                                core_cfg_addr <= packet[1][5:0];
                                core_cfg_data <= packet_data; core_cfg_valid <= 1;
                            end else reject_pending <= 1;
                        end else if (decoded_pose) begin
                            // Forward all payload bits. Bad magic/version must abort
                            // the transaction in the owner, never reuse an old bitmap.
                            if (core_cfg_ready && !core_cfg_valid) begin
                                core_cfg_addr <= 6'h3e;
                                core_cfg_data <= (decoded_visual ? packet_data[31:16]==16'h5649 :
                                  packet_data[31:16]==16'h504f) ? packet_data : 32'b0;
                                core_cfg_valid <= 1;
                            end else reject_pending <= 1;
                        end else if (decoded_begin) begin
                            if (decoded_begin_magic) begin
                                if (core_cfg_ready) begin
                                    core_cfg_addr <= 6'h3f; core_cfg_data <= packet_data; core_cfg_valid <= 1;
                                end else reject_pending <= 1;
                            end else begin
                                packet_error_count <= increment_saturated(packet_error_count);
                                reject_pending <= 1;
                                core_cfg_addr <= 6'h3f; core_cfg_data <= 0; core_cfg_valid <= 1;
                            end
                        end else if (decoded_commit && decoded_commit_magic) begin
                            if (core_cfg_commit_ready && !core_cfg_valid) core_cfg_commit <= 1;
                            else reject_pending <= 1;
                        end else if (decoded_aux) begin
                            // Ordered C127 profile transaction: E8 context,
                            // E7 flags, 40..63 four row-major C matrices,
                            // E9 matching context. EA invalidates, EB sets modes.
                            // An auxiliary error does not overwrite active C.
                            if (!aux_valid || aux_ready) begin
                                aux_addr <= packet[1] == 8'hec ? 6'h39 : packet[1] >= 8'he7 ?
                                            (6'h3b + packet[1] - 8'he7) :
                                            (packet[1] - 8'h40);
                                aux_data <= packet_data; aux_valid <= 1;
                            end else reject_pending <= 1;
                        end else if (decoded_sccb) begin
                            // Independent sensor-register diagnostic namespace.
                        end else if (decoded_enhance) begin
                            // Validate all 32 data bits before updating the mode.
                            if (decoded_mode_range) begin
                                enhance_mode <= packet_data[2:0];
                                enhance_valid <= 1;
                            end else packet_error_count <= increment_saturated(packet_error_count);
                        end else if (decoded_detail) begin
                            if (packet_data[31:1] == 0) begin
                                detail_enable <= packet_data[0]; detail_valid <= 1;
                            end else packet_error_count <= increment_saturated(packet_error_count);
                        end else packet_error_count <= increment_saturated(packet_error_count);
                    end else begin
                        crc_error_count <= increment_saturated(crc_error_count);
                        // Abort has priority over a buffered old parameter.
                        // Hold it until ready, so a receiver error cannot leave
                        // an older transaction's written bitmap reusable.
                        core_cfg_addr <= 6'h3f; core_cfg_data <= 0; core_cfg_valid <= 1;
                        aux_addr <= 6'h3a; aux_data <= 0; aux_valid <= 1;
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
                    core_cfg_addr <= 6'h3f; core_cfg_data <= 0; core_cfg_valid <= 1;
                    aux_addr <= 6'h3a; aux_data <= 0; aux_valid <= 1;
                end else idle_cycles <= idle_cycles+32'd1;
            end else idle_cycles <= 0;
        end
    end
endmodule
