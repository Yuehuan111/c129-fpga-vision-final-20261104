`timescale 1ns/1ps
// One OV5640 SCCB register transaction. All pad outputs are open-drain enables:
// assign pad = drive_low ? 1'b0 : 1'bz. External pull-ups are required.
// No hardware ownership is assumed: enable must only assert after boot has
// relinquished both pads. Revoking enable aborts and releases both pads.
// Default nominal SCL is 100 kHz; synchronized SCL-high detection adds a small
// conservative delay and supports clock stretching. No generated fabric clock.
module sccb_runtime_master #(
    parameter integer CLK_HZ = 100000000,
    parameter integer SCCB_HZ = 100000,
    parameter integer TIMEOUT_CYCLES = CLK_HZ/100,
    parameter [6:0] DEVICE_ADDRESS = 7'h3c
) (
    input wire clk, rst_n, enable,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire cmd_write,
    input wire [15:0] cmd_register,
    input wire [7:0] cmd_data,
    input wire scl_in, sda_in,
    output reg scl_drive_low, sda_drive_low,
    output reg busy, done, result_valid,
    output reg [31:0] transaction_seq, result_seq,
    output reg result_write,
    output reg [15:0] result_register,
    output reg [7:0] result_data,
    // 0=success, 1=NACK, 2=transaction timeout, 3=ownership revoked.
    output reg [3:0] result_error,
    // NACK phase: 1=write ID, 2=register MSB, 3=register LSB,
    // 4=write value, 5=read ID. Read's final master NACK is normal.
    output reg [3:0] nack_phase
);
    localparam integer QUARTER = (CLK_HZ + 4*SCCB_HZ-1)/(4*SCCB_HZ);
    localparam [4:0] IDLE=0, BUS_FREE=1, START_HOLD=2,
        BIT_LOW_A=3, BIT_LOW_B=4, BIT_RISE=5, BIT_HIGH_A=6,
        BIT_HIGH_B=7, STOP_LOW=8, STOP_SETUP=9, STOP_RISE=10,
        STOP_HOLD=11, STOP_FREE=12;
    reg [4:0] state;
    reg [31:0] delay_count, timeout_count;
    reg [2:0] phase, bit_index;
    reg [7:0] tx_byte;
    reg ack_bit, read_byte, restart_read;
    (* syn_preserve=1, syn_srlstyle="registers" *) reg scl_meta, scl_sync;
    (* syn_preserve=1, syn_srlstyle="registers" *) reg sda_meta, sda_sync;
    assign cmd_ready = enable && !busy && (state == IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin scl_meta<=1; scl_sync<=1; sda_meta<=1; sda_sync<=1; end
        else begin scl_meta<=scl_in; scl_sync<=scl_meta;
                   sda_meta<=sda_in; sda_sync<=sda_meta; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=IDLE; delay_count<=0; timeout_count<=0;
            phase<=0; bit_index<=7; tx_byte<=0; ack_bit<=0;
            read_byte<=0; restart_read<=0;
            scl_drive_low<=0; sda_drive_low<=0; busy<=0; done<=0;
            result_valid<=0; transaction_seq<=0; result_seq<=0;
            result_write<=0; result_register<=0; result_data<=0;
            result_error<=0; nack_phase<=0;
        end else begin
            done<=0;
            if (busy) timeout_count<=timeout_count+1'b1;
            // Timeout covers START, every ACK/bit, STOP and both read phases.
            // Release rather than driving high, even if another device holds low.
            if (busy && (!enable || timeout_count>=TIMEOUT_CYCLES-1)) begin
                state<=IDLE; scl_drive_low<=0; sda_drive_low<=0; busy<=0;
                done<=1; result_valid<=1; result_seq<=transaction_seq;
                result_error<=enable ? 4'd2 : 4'd3;
            end else case (state)
                IDLE: begin
                    scl_drive_low<=0; sda_drive_low<=0;
                    if (cmd_valid && cmd_ready) begin
                        busy<=1; result_valid<=0; timeout_count<=0;
                        transaction_seq<=transaction_seq+1'b1;
                        result_write<=cmd_write; result_register<=cmd_register;
                        result_data<=cmd_write ? cmd_data : 8'b0;
                        result_error<=0; nack_phase<=0;
                        phase<=0; tx_byte<={DEVICE_ADDRESS,1'b0};
                        read_byte<=0; ack_bit<=0; bit_index<=7;
                        restart_read<=0; delay_count<=2*QUARTER-1;
                        state<=BUS_FREE;
                    end
                end
                BUS_FREE: begin
                    if (!scl_sync || !sda_sync) delay_count<=2*QUARTER-1;
                    else if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin
                        sda_drive_low<=1; delay_count<=2*QUARTER-1;
                        state<=START_HOLD;
                    end
                end
                START_HOLD: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin scl_drive_low<=1; delay_count<=QUARTER-1; state<=BIT_LOW_A; end
                BIT_LOW_A: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin
                        // Changes SDA only after SCL has been low for a quarter.
                        // Reads and every ACK slot release SDA (final read NACK).
                        sda_drive_low<=(!read_byte && !ack_bit) ? !tx_byte[bit_index] : 1'b0;
                        delay_count<=QUARTER-1; state<=BIT_LOW_B;
                    end
                BIT_LOW_B: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin scl_drive_low<=0; state<=BIT_RISE; end
                BIT_RISE: if (scl_sync) begin delay_count<=QUARTER-1; state<=BIT_HIGH_A; end
                BIT_HIGH_A: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin
                        if (ack_bit && !read_byte && sda_sync) begin
                            result_error<=1; nack_phase<={1'b0,phase}+1'b1;
                        end
                        if (read_byte && !ack_bit) result_data[bit_index]<=sda_sync;
                        delay_count<=QUARTER-1; state<=BIT_HIGH_B;
                    end
                BIT_HIGH_B: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin
                        scl_drive_low<=1; delay_count<=QUARTER-1;
                        if (!ack_bit) begin
                            if (bit_index!=0) bit_index<=bit_index-1'b1;
                            else ack_bit<=1;
                            state<=BIT_LOW_A;
                        end else begin
                            ack_bit<=0; bit_index<=7;
                            if (result_error!=0 || phase==3 || phase==5) begin
                                restart_read<=0; state<=STOP_LOW;
                            end else if (phase==2 && !result_write) begin
                                // OV5640 SCCB random read: pointer write, STOP,
                                // a new START/read ID, one byte, NACK and STOP.
                                restart_read<=1; state<=STOP_LOW;
                            end else begin
                                phase<=phase+1'b1; state<=BIT_LOW_A;
                                case (phase)
                                    0: tx_byte<=result_register[15:8];
                                    1: tx_byte<=result_register[7:0];
                                    2: tx_byte<=result_data;
                                    4: begin tx_byte<=8'hff; read_byte<=1; end
                                    default: tx_byte<=8'hff;
                                endcase
                            end
                        end
                    end
                STOP_LOW: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin sda_drive_low<=1; delay_count<=QUARTER-1; state<=STOP_SETUP; end
                STOP_SETUP: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin scl_drive_low<=0; state<=STOP_RISE; end
                STOP_RISE: if (scl_sync) begin delay_count<=2*QUARTER-1; state<=STOP_HOLD; end
                STOP_HOLD: if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else begin sda_drive_low<=0; delay_count<=2*QUARTER-1; state<=STOP_FREE; end
                STOP_FREE: begin
                    // Verify that STOP actually released the bus. Stuck SDA/SCL
                    // is reported by the same transaction watchdog.
                    if (!scl_sync || !sda_sync) delay_count<=2*QUARTER-1;
                    else if (delay_count!=0) delay_count<=delay_count-1'b1;
                    else if (restart_read) begin
                        restart_read<=0; phase<=4; read_byte<=0;
                        tx_byte<={DEVICE_ADDRESS,1'b1}; ack_bit<=0; bit_index<=7;
                        sda_drive_low<=1; delay_count<=2*QUARTER-1; state<=START_HOLD;
                    end else begin
                        state<=IDLE; busy<=0; done<=1; result_valid<=1;
                        result_seq<=transaction_seq;
                    end
                end
                default: begin state<=IDLE; scl_drive_low<=0; sda_drive_low<=0; busy<=0; end
            endcase
        end
    end
endmodule
