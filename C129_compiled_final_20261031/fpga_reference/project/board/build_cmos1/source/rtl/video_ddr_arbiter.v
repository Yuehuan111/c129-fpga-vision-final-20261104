// Reserve service while the display fills a line. Each response now writes
// all sixteen RAM lanes in one clock; there is no per-pixel unpack interval.
// An already accepted transaction is completed by ddr_single_beat first.
module video_ddr_arbiter(
 input wire display_req,display_service,writer_req,core_req,bus_ready,
 output wire bus_req,output wire [1:0] grant,
 output wire display_ready,writer_ready,core_ready
);
 assign grant=(display_req||display_service)?2'd0:writer_req?2'd1:2'd2;
 assign bus_req=display_req||(!display_service&&(writer_req||core_req));
 assign display_ready=bus_ready&&bus_req&&(grant==0);
 assign writer_ready=bus_ready&&bus_req&&(grant==1);
 assign core_ready=bus_ready&&bus_req&&(grant==2);
endmodule
