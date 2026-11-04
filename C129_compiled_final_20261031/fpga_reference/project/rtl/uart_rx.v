`timescale 1ns/1ps
// Asynchronous UART receiver: 8 data bits, no parity, one stop bit, LSB first.
// CLK_HZ/BAUD must provide at least four clocks per serial bit. Divider rounding
// must keep the baud error within the sender/receiver timing budget.
module uart_rx #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid,
    output reg        framing_error,
    output wire       busy
);
    localparam integer BIT_CLOCKS = (CLK_HZ + BAUD/2) / BAUD;
    localparam integer HALF_CLOCKS = BIT_CLOCKS / 2;
    localparam [2:0] IDLE = 0, START = 1, BITS = 2, STOP = 3, WAIT_HIGH = 4;
    (* syn_preserve=1, syn_srlstyle="registers" *) reg rx_meta, rx_sync;
    reg [2:0] state, bit_index;
    reg [31:0] countdown;
    reg [7:0] shift_data;
    assign busy = (state != IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta <= 1; rx_sync <= 1; end
        else begin rx_meta <= rx; rx_sync <= rx_meta; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; bit_index <= 0; countdown <= 0; shift_data <= 0;
            data <= 0; valid <= 0; framing_error <= 0;
        end else begin
            valid <= 0; framing_error <= 0;
            case (state)
                IDLE: if (!rx_sync) begin state <= START; countdown <= HALF_CLOCKS-1; end
                START: begin
                    if (countdown != 0) countdown <= countdown-1;
                    else if (!rx_sync) begin
                        state <= BITS; bit_index <= 0; countdown <= BIT_CLOCKS-1;
                    end else state <= IDLE; // Reject a short low glitch.
                end
                BITS: begin
                    if (countdown != 0) countdown <= countdown-1;
                    else begin
                        shift_data[bit_index] <= rx_sync;
                        countdown <= BIT_CLOCKS-1;
                        if (bit_index == 7) state <= STOP;
                        else bit_index <= bit_index+1'b1;
                    end
                end
                STOP: begin
                    if (countdown != 0) countdown <= countdown-1;
                    else if (rx_sync) begin
                        data <= shift_data; valid <= 1; state <= IDLE;
                    end else begin
                        framing_error <= 1; state <= WAIT_HIGH;
                    end
                end
                WAIT_HIGH: if (rx_sync) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
