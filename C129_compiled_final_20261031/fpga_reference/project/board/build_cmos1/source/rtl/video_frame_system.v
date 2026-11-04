// Capture one validated frame, freeze it, correct it, publish at a display VSYNC.
// RGB565 source is region 0. Output buffers start at 4 MiB and 8 MiB.
// Processing throughput is deliberately independent of HDMI refresh rate.
module video_frame_system #(
 parameter IMG_WIDTH=1280,IMG_HEIGHT=720,FIFO_ADDR_BITS=11,
 parameter [31:0] OUTPUT0_BASE=32'h00400000,OUTPUT1_BASE=32'h00800000
)(
 input wire clk,rst_n,cam_clk,cam_rst_n,pix_clk,pix_rst_n,
 input wire cam_valid,input wire[15:0] cam_rgb565,input wire cam_sof,
 input wire cam_frame_done,cam_frame_drop,
 input wire video_hs,video_vs,video_de,input wire[15:0] video_x,video_y,
 output wire hdmi_hs,hdmi_vs,hdmi_de,output wire[15:0] hdmi_rgb565,
 input wire cfg_valid,input wire[4:0] cfg_addr,input wire[31:0] cfg_data,
 input wire cfg_commit,output wire cfg_ready,cfg_commit_ready,
 output wire[27:0] axi_awaddr,output wire[3:0] axi_awlen,
 output wire axi_awvalid,input wire axi_awready,
 output wire[255:0] axi_wdata,output wire[31:0] axi_wstrb,
 input wire axi_wready,axi_wlast,
 output wire[27:0] axi_araddr,output wire[3:0] axi_arlen,
 output wire axi_arvalid,input wire axi_arready,
 input wire[255:0] axi_rdata,input wire axi_rvalid,axi_rlast,
 output reg[31:0] captured_frames,processed_frames,repeated_frames,dropped_frames,
 output wire[31:0] display_underflow_lines,output wire[31:0] capture_overflow_frames,
 output wire[31:0] camera_skipped_frames,
 output reg error_sticky,output wire processing,output wire[15:0] config_version
);
 localparam PIXELS=IMG_WIDTH*IMG_HEIGHT;
 localparam WAIT_CONFIG=0,CAPTURE=1,CORE_START=2,PROCESS=3,WAIT_SWAP=4;
 reg[2:0] state;
 reg capture_request,have_config;
 wire cap_valid,cap_ready;wire[17:0] cap_data;
 video_capture_cdc #(.FIFO_ADDR_BITS(FIFO_ADDR_BITS)) capture(
  .cam_clk(cam_clk),.cam_rst_n(cam_rst_n),.pix_valid(cam_valid),.rgb565(cam_rgb565),
  .sof(cam_sof),.frame_valid_done(cam_frame_done),.frame_drop(cam_frame_drop),
  .request_toggle(capture_request),.clk(clk),.rst_n(rst_n),
  .valid(cap_valid),.data(cap_data),.ready(cap_ready),.overflow_frames(capture_overflow_frames),
  .skipped_frames(camera_skipped_frames));

 reg front_valid,front_bank,pending_valid,pending_bank,back_bank;
 wire[31:0] front_base=front_bank?OUTPUT1_BASE:OUTPUT0_BASE;
 wire[31:0] back_base=back_bank?OUTPUT1_BASE:OUTPUT0_BASE;
 wire display_event,display_req,display_req_ready,display_rsp,display_rsp_ready,display_service;
 wire[31:0] display_addr;wire[255:0] display_data;
 video_display_ddr #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) display(
  .clk(clk),.rst_n(rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),
  .hs(video_hs),.vs(video_vs),.de(video_de),.x(video_x),.y(video_y),
  .front_valid(front_valid),.front_base(front_base),.frame_event(display_event),
  .service_active(display_service),
  .req_valid(display_req),.req_addr(display_addr),.req_ready(display_req_ready),
  .rsp_valid(display_rsp),.rsp_data(display_data),.rsp_ready(display_rsp_ready),
  .out_hs(hdmi_hs),.out_vs(hdmi_vs),.out_de(hdmi_de),.out_rgb565(hdmi_rgb565),
  .underflow_lines(display_underflow_lines));

 wire core_busy,core_done,core_error,core_rd_valid,core_rd_ready;
 wire[31:0] core_rd_addr;wire core_rsp_valid,core_rsp_ready;wire[15:0] core_rsp_data;
 wire core_out_valid,core_out_ready,core_out_last;wire[15:0] core_out_rgb;
 wire[15:0] core_x,core_y;
 correction_core #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) corrector(
  .clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_addr(cfg_addr),.cfg_data(cfg_data),
  .cfg_commit(cfg_commit),.cfg_ready(cfg_ready),.cfg_commit_ready(cfg_commit_ready),
  .start(state==CORE_START),.frame_id(captured_frames),.busy(core_busy),
  .done(core_done),.error(core_error),.config_version(config_version),
  .rd_valid(core_rd_valid),.rd_ready(core_rd_ready),.rd_addr(core_rd_addr),
  .rd_rsp_valid(core_rsp_valid),.rd_rsp_ready(core_rsp_ready),.rd_rsp_data(core_rsp_data),
  .out_valid(core_out_valid),.out_ready(core_out_ready),.out_rgb565(core_out_rgb),
  .out_x(core_x),.out_y(core_y),.out_last(core_out_last));
 assign processing=(state==PROCESS || state==CORE_START);

 reg[255:0] packed_pixels;
 reg[4:0] pack_count;
 reg[31:0] pixel_count,word_count;
 reg writer_wait,end_seen,end_good,core_finished,last_seen;
 wire writer_req=(state==CAPTURE || state==PROCESS) && !writer_wait &&
                ((pack_count==16)||((end_seen||core_finished)&&pack_count!=0));
 wire[31:0] writer_addr=(state==CAPTURE?32'b0:back_base)+(word_count<<5);
 wire[31:0] writer_mask=(pack_count==16)?32'hffffffff:((32'h1<<(pack_count*2))-1'b1);
 wire writer_ready,writer_rsp;
 // The bridge copies the accepted beat into its own wd register. Refill the
 // packer while that write is outstanding instead of stalling the camera FIFO.
 assign cap_ready=(state==CAPTURE)&&!writer_req&&(pack_count<16);
 assign core_out_ready=(state==PROCESS)&&!writer_req&&(pack_count<16);

 // Priority: display line prefetch, sequential capture/output writer, correction read.
 // Exactly one transaction is outstanding, so response ownership cannot interleave.
 wire bus_ready,bus_rsp_valid,bus_rsp_ready;wire[255:0] bus_rsp_data;
 reg[1:0] owner;reg[3:0] read_lane;
 wire[1:0] grant;
 wire bus_req;
 video_ddr_arbiter arbiter(
  .display_req(display_req),.display_service(display_service),
  .writer_req(writer_req),.core_req(core_rd_valid),.bus_ready(bus_ready),
  .bus_req(bus_req),.grant(grant),.display_ready(display_req_ready),
  .writer_ready(writer_ready),.core_ready(core_rd_ready));
 wire[31:0] bus_addr=(grant==0)?display_addr:(grant==1)?writer_addr:
                     {core_rd_addr[30:4],5'b0};
 assign display_rsp=bus_rsp_valid&&(owner==0);
 assign display_data=bus_rsp_data;
 assign writer_rsp=bus_rsp_valid&&(owner==1);
 assign core_rsp_valid=bus_rsp_valid&&(owner==2);
 assign core_rsp_data=bus_rsp_data[read_lane*16+:16];
 assign bus_rsp_ready=(owner==0)?display_rsp_ready:(owner==1)?1'b1:core_rsp_ready;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin owner<=0;read_lane<=0;end
 else if(bus_req&&bus_ready) begin owner<=grant;read_lane<=core_rd_addr[3:0];end
 ddr_single_beat memory_bridge(
  .clk(clk),.rst_n(rst_n),.req_valid(bus_req),.req_write(grant==1),.req_addr(bus_addr),
  .req_wdata(packed_pixels),.req_wstrb(writer_mask),.req_ready(bus_ready),
  .rsp_valid(bus_rsp_valid),.rsp_data(bus_rsp_data),.rsp_ready(bus_rsp_ready),
  .axi_awaddr(axi_awaddr),.axi_awlen(axi_awlen),.axi_awvalid(axi_awvalid),.axi_awready(axi_awready),
  .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wready(axi_wready),.axi_wlast(axi_wlast),
  .axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
  .axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid),.axi_rlast(axi_rlast));

 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=WAIT_CONFIG;capture_request<=0;have_config<=0;front_valid<=0;
  front_bank<=1;pending_valid<=0;pending_bank<=0;back_bank<=0;
  packed_pixels<=0;pack_count<=0;pixel_count<=0;word_count<=0;writer_wait<=0;
  end_seen<=0;end_good<=0;core_finished<=0;last_seen<=0;
  captured_frames<=0;processed_frames<=0;repeated_frames<=0;dropped_frames<=0;error_sticky<=0;
 end else begin
  if(cfg_commit&&cfg_commit_ready) have_config<=1;
  if(processing&&core_error) error_sticky<=1;
  if(display_event) begin
   if(pending_valid) begin front_bank<=pending_bank;front_valid<=1;pending_valid<=0;end
   else if(front_valid) repeated_frames<=repeated_frames+1'b1;
  end
  if(writer_req&&writer_ready) begin
   writer_wait<=1;pack_count<=0;packed_pixels<=0;word_count<=word_count+1'b1;
  end
  if(writer_rsp) writer_wait<=0;
  case(state)
   WAIT_CONFIG: if(have_config) begin state<=CAPTURE;capture_request<=~capture_request;end
   CAPTURE: begin
    if(cap_valid&&cap_ready) begin
     if(cap_data[17:16]==0) begin
      if(pixel_count<PIXELS) begin
       packed_pixels[pack_count*16+:16]<=cap_data[15:0];pack_count<=pack_count+1'b1;
      end
      pixel_count<=pixel_count+1'b1;
     end else begin end_seen<=1;end_good<=cap_data[17:16]==1;end
    end
    if(end_seen&&!writer_wait&&pack_count==0) begin
     pixel_count<=0;word_count<=0;end_seen<=0;core_finished<=0;last_seen<=0;
     if(end_good&&pixel_count==PIXELS) begin
      captured_frames<=captured_frames+1'b1;state<=CORE_START;
     end else begin dropped_frames<=dropped_frames+1'b1;capture_request<=~capture_request;end
    end
   end
   // Commit may block the core's start in this cycle. Keep requesting until
   // busy acknowledges acceptance, rather than waiting forever for done.
   CORE_START: if(core_busy) state<=PROCESS;
   PROCESS: begin
    if(core_out_valid&&core_out_ready) begin
     packed_pixels[pack_count*16+:16]<=core_out_rgb;pack_count<=pack_count+1'b1;
     pixel_count<=pixel_count+1'b1;
     if(core_out_last) last_seen<=1;
    end
    if(core_done) core_finished<=1;
    if(core_finished&&!writer_wait&&pack_count==0) begin
     if(pixel_count==PIXELS&&last_seen) begin
      pending_bank<=back_bank;pending_valid<=1;processed_frames<=processed_frames+1'b1;
      state<=WAIT_SWAP;
     end else begin error_sticky<=1;state<=WAIT_CONFIG;end
     pixel_count<=0;word_count<=0;core_finished<=0;last_seen<=0;
    end
   end
   WAIT_SWAP: if(display_event&&pending_valid) begin
    back_bank<=~pending_bank;capture_request<=~capture_request;state<=CAPTURE;
   end
   default: state<=WAIT_CONFIG;
  endcase
 end
endmodule
