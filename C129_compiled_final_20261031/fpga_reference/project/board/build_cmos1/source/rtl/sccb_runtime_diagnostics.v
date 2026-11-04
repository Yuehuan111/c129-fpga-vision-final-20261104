`timescale 1ns/1ps
// Integration wrapper only; it does not instantiate or drive a UART transmitter.
// Gate the existing boot pad drivers with !runtime_owner, then connect the
// runtime drive_low enables to the SAME pads. See INTEGRATION.md for exact mux.
module sccb_runtime_diagnostics #(
    parameter integer CLK_HZ=100000000,
    parameter integer BAUD=115200,
    parameter integer SCCB_HZ=100000,
    parameter integer TIMEOUT_CYCLES=CLK_HZ/100,
    parameter integer HANDOFF_IDLE_CYCLES=CLK_HZ/100000
) (
    input wire clk,rst_n,uart_rx,boot_config_done,
    input wire scl_in,sda_in,
    output reg runtime_owner,
    output wire scl_drive_low,sda_drive_low,
    output wire busy,done,
    // Eight core-clock-domain status words, least-significant word first.
    // word0: owner/busy/result_valid/write in bits0..3; error[7:4],
    // nack_phase[11:8], last_reject_reason[15:12], boot_done bit16,
    // SCL/SDA synchronized levels bits17/18. Other bits are zero.
    // word1 accepted sequence; word2 completed sequence;
    // word3 {00,data[7:0],register[15:0]}; word4 rejected count;
    // word5 CRC errors; word6 UART framing errors; word7 packet errors.
    output wire [255:0] status_words
);
    (* syn_preserve=1, syn_srlstyle="registers" *) reg boot_meta,boot_sync;
    (* syn_preserve=1, syn_srlstyle="registers" *) reg scl_meta,scl_sync,sda_meta,sda_sync;
    reg [31:0] idle_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            boot_meta<=0;boot_sync<=0;scl_meta<=1;scl_sync<=1;sda_meta<=1;sda_sync<=1;
            runtime_owner<=0;idle_count<=0;
        end else begin
            boot_meta<=boot_config_done;boot_sync<=boot_meta;
            scl_meta<=scl_in;scl_sync<=scl_meta;sda_meta<=sda_in;sda_sync<=sda_meta;
            if (!boot_sync) begin runtime_owner<=0;idle_count<=0;end
            else if (!runtime_owner) begin
                if (!scl_sync || !sda_sync) idle_count<=0;
                else if (idle_count>=HANDOFF_IDLE_CYCLES-1) begin runtime_owner<=1;idle_count<=0;end
                else idle_count<=idle_count+1'b1;
            end
        end
    end
    wire cmd_valid,cmd_ready,cmd_write,result_valid,result_write;
    wire [15:0] cmd_register,result_register;
    wire [7:0] cmd_data,result_data;
    wire [31:0] seq,result_seq,rejected,crc_errors,framing_errors,packet_errors;
    wire [3:0] result_error,nack_phase,last_reject_reason;
    uart_sccb_adapter #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver (
        .clk(clk),.rst_n(rst_n),.uart_rx(uart_rx),.boot_ready(runtime_owner),
        .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_write(cmd_write),
        .cmd_register(cmd_register),.cmd_data(cmd_data),.accepted_count(),
        .rejected_count(rejected),.crc_error_count(crc_errors),
        .framing_error_count(framing_errors),.packet_error_count(packet_errors),
        .ignored_count(),.last_reject_reason(last_reject_reason)
    );
    sccb_runtime_master #(.CLK_HZ(CLK_HZ),.SCCB_HZ(SCCB_HZ),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) master (
        .clk(clk),.rst_n(rst_n),.enable(runtime_owner),.cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),.cmd_write(cmd_write),.cmd_register(cmd_register),.cmd_data(cmd_data),
        .scl_in(scl_in),.sda_in(sda_in),.scl_drive_low(scl_drive_low),.sda_drive_low(sda_drive_low),
        .busy(busy),.done(done),.result_valid(result_valid),.transaction_seq(seq),.result_seq(result_seq),
        .result_write(result_write),.result_register(result_register),.result_data(result_data),
        .result_error(result_error),.nack_phase(nack_phase)
    );
    assign status_words={packet_errors,framing_errors,crc_errors,rejected,
        8'b0,result_data,result_register,result_seq,seq,
        13'b0,sda_sync,scl_sync,boot_sync,last_reject_reason,nack_phase,
        result_error,result_write,result_valid,busy,runtime_owner};
endmodule
