// MES50HP / Logos PGL50H-6IFBG484. 50 MHz oscillator, 100 MHz DDR UI.
// Wide-FOV720p30 RGB565/tiled-fisheye candidate;720p60 HDMI refresh. Requires board validation.
module camera_board50_top#(
    parameter CAMERA_CHANNEL=1,
    parameter VSYNC_ACTIVE_LEVEL=1,
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8
)(
	input sys_clk, // 50 MHz on MES50HP, pin P20
    input uart_rx,
    input geometry_key_n, // KEY1 K18: raw/restore
    input scan_key_n, // KEY2 L15: plane scan
    input mirror_key_n, // KEY3 J17: horizontal output mirror
    output uart_tx,
    output result_led,
    output error_led,
    output capture_led,
//OV5640
    output  [1:0]                        cmos_init_done       ,//OV5640寄存器初始化完成
    //coms1	
    inout                                cmos1_scl            ,//cmos1 i2c 
    inout                                cmos1_sda            ,//cmos1 i2c 
    input                                cmos1_vsync          ,//cmos1 vsync
    input                                cmos1_href           ,//cmos1 hsync refrence,data valid
    input                                cmos1_pclk           ,//cmos1 pxiel clock
    input   [7:0]                        cmos1_data           ,//cmos1 data
    output                               cmos1_reset          ,//cmos1 reset
    //coms2
    inout                                cmos2_scl            ,//cmos2 i2c 
    inout                                cmos2_sda            ,//cmos2 i2c 
    input                                cmos2_vsync          ,//cmos2 vsync
    input                                cmos2_href           ,//cmos2 hsync refrence,data valid
    input                                cmos2_pclk           ,//cmos2 pxiel clock
    input   [7:0]                        cmos2_data           ,//cmos2 data
    output                               cmos2_reset          ,//cmos2 reset
//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成
//HDMI_OUT
    output                               pix_clk                   ,//pixclk                           
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         
);
///////////////////////////////////////////////////////////////////////////////////
localparam IMG_WIDTH=1280, IMG_HEIGHT=720;
localparam CORE_CLK_HZ=100000000;
wire cfg_clk,clk_25M,locked,core_clk,pll_lock;
wire [27:0] axi_awaddr,axi_araddr;
wire [3:0] axi_awlen,axi_arlen,axi_rid;
wire [3:0] axi_awuser_id=0,axi_aruser_id=0;
wire axi_awready,axi_awvalid,axi_wready,axi_wusero_last,axi_arready,axi_arvalid,axi_rlast,axi_rvalid;
wire [255:0] axi_wdata,axi_rdata;
wire [31:0] axi_wstrb;
wire pll_startup_rst;
pll_startup_reset pll_reset(.clk(sys_clk),.reset(pll_startup_rst));
pll u_pll(.clkin1(sys_clk),.pll_rst(pll_startup_rst),.clkout0(pix_clk),.clkout1(cfg_clk),.clkout2(clk_25M),.pll_lock(locked));
reg [15:0] reset_delay=0;
always @(posedge cfg_clk) if(!locked) reset_delay<=0;else if(reset_delay!=10000) reset_delay<=reset_delay+1'b1;
assign rstn_out=(reset_delay==10000);
wire init_over_tx;
ms72xx_ctl hdmi_setup(.clk(cfg_clk),.rst_n(rstn_out),.init_over_tx(init_over_tx),.init_over_rx(),.iic_tx_scl(iic_tx_scl),.iic_tx_sda(iic_tx_sda),.iic_scl(),.iic_sda());
assign hdmi_int_led=init_over_tx;
wire initial_en,camera_reset,cam_config_done;
wire selected_pclk=(CAMERA_CHANNEL==1)?cmos1_pclk:cmos2_pclk;
wire selected_vsync=(CAMERA_CHANNEL==1)?cmos1_vsync:cmos2_vsync;
wire selected_href=(CAMERA_CHANNEL==1)?cmos1_href:cmos2_href;
wire [7:0] selected_data=(CAMERA_CHANNEL==1)?cmos1_data:cmos2_data;
wire selected_scl=(CAMERA_CHANNEL==1)?cmos1_scl:cmos2_scl;
wire selected_sda=(CAMERA_CHANNEL==1)?cmos1_sda:cmos2_sda;
wire sccb_runtime_owner,sccb_scl_low,sccb_sda_low;
wire[255:0] sccb_status;
power_on_delay camera_power(.clk_50M(sys_clk),.reset_n(locked),
 .camera1_rstn(camera_reset),.camera2_rstn(),.camera_pwnd(),.initial_en(initial_en));
generate if(CAMERA_CHANNEL==1) begin: sensor1
 wire boot_scl_high,boot_sda_low;
 assign cmos1_reset=camera_reset;
 // Preserve the validated push-pull boot SCL. Runtime SCL/SDA are open drain.
 assign cmos1_scl=sccb_runtime_owner?(sccb_scl_low?1'b0:1'bz):boot_scl_high;
 assign cmos1_sda=(sccb_runtime_owner?sccb_sda_low:boot_sda_low)?1'b0:1'bz;
 assign cmos2_reset=0; assign cmos2_scl=1'bz; assign cmos2_sda=1'bz;
 assign cmos_init_done={1'b0,cam_config_done};
 reg_config camera_setup(.clk_25M(clk_25M),.camera_rstn(camera_reset&&initial_en),.initial_en(initial_en),
 .i2c_sclk(boot_scl_high),.i2c_sdat_in(cmos1_sda),.i2c_sdat_drive_low(boot_sda_low),
 .reg_conf_done(cam_config_done),.reg_index(),.clock_20k());
end else begin: sensor2
 wire boot_scl_high,boot_sda_low;
 assign cmos2_reset=camera_reset;
 assign cmos2_scl=sccb_runtime_owner?(sccb_scl_low?1'b0:1'bz):boot_scl_high;
 assign cmos2_sda=(sccb_runtime_owner?sccb_sda_low:boot_sda_low)?1'b0:1'bz;
 assign cmos1_reset=0; assign cmos1_scl=1'bz; assign cmos1_sda=1'bz;
 assign cmos_init_done={cam_config_done,1'b0};
 reg_config camera_setup(.clk_25M(clk_25M),.camera_rstn(camera_reset&&initial_en),.initial_en(initial_en),
 .i2c_sclk(boot_scl_high),.i2c_sdat_in(cmos2_sda),.i2c_sdat_drive_low(boot_sda_low),
 .reg_conf_done(cam_config_done),.reg_index(),.clock_20k());
end endgenerate
wire core_rst_n,core_system_rst_n,core_status_rst_n,cam_rst_n,pix_rst_n;
video_core_reset_sync cr(.clk(core_clk),.async_n(ddr_init_done&&locked),
 .rst_n(core_rst_n),.system_rst_n(core_system_rst_n),.status_rst_n(core_status_rst_n));
video_reset_sync pr(.clk(pix_clk),.async_n(ddr_init_done&&locked),.rst_n(pix_rst_n));
video_reset_sync vr(.clk(selected_pclk),.async_n(ddr_init_done&&locked&&cam_config_done),.rst_n(cam_rst_n));
sccb_runtime_diagnostics #(.CLK_HZ(CORE_CLK_HZ)) sensor_registers(
 .clk(core_clk),.rst_n(core_rst_n),.uart_rx(uart_rx),.boot_config_done(cam_config_done),
 .scl_in(selected_scl),.sda_in(selected_sda),.runtime_owner(sccb_runtime_owner),
 .scl_drive_low(sccb_scl_low),.sda_drive_low(sccb_sda_low),.busy(),.done(),.status_words(sccb_status));
wire cam_valid,cam_sof,cam_done,cam_drop;
wire [15:0] cam_rgb,cam_x,cam_y;wire cam_error;
wire [31:0] dvp_drops;
// Keep existing UART diagnostic field names and Gray counter instance names.
// "raw_valid_frames" now counts complete sensor RGB565 frames before warp.
wire raw_done=cam_done,raw_error,bayer_error=1'b0;
reg[31:0] raw_valid_frames_cam;
wire[31:0] raw_valid_frames_core;
always @(posedge selected_pclk or negedge cam_rst_n)
 if(!cam_rst_n) raw_valid_frames_cam<=0;
 else if(raw_done) raw_valid_frames_cam<=raw_valid_frames_cam+1'b1;
counter_cdc raw_frame_stats(
 .source_clk(selected_pclk),.source_rst_n(cam_rst_n),.source_binary(raw_valid_frames_cam),
 .destination_clk(core_clk),.destination_rst_n(core_rst_n),.destination_binary(raw_valid_frames_core));
dvp_rgb565_rx #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.VSYNC_ACTIVE_LEVEL(VSYNC_ACTIVE_LEVEL),.SWAP_RB(1)) dvp(
 .pclk(selected_pclk),.rst_n(cam_rst_n),.vsync(selected_vsync),.href(selected_href),.data(selected_data),
 .pix_valid(cam_valid),.rgb565(cam_rgb),.x(cam_x),.y(cam_y),.sof(cam_sof),.eol(),.eof(),
 .frame_valid_done(cam_done),.frame_drop(cam_drop),.frame_error(raw_error),.drop_count(dvp_drops));
assign cam_error=raw_error;
wire diagnostic_rst_n;
wire[479:0] input_diagnostics;
video_reset_sync diagnostic_reset(.clk(selected_pclk),.async_n(ddr_init_done&&locked),.rst_n(diagnostic_rst_n));
video_input_diagnostics input_probe(
 .core_clk(core_clk),.core_rst_n(core_rst_n),.pclk(selected_pclk),.source_rst_n(diagnostic_rst_n),
 .vsync(selected_vsync),.href(selected_href),.data(selected_data),
 .raw_error(raw_error),.bayer_error(bayer_error),.camera_reset_released(cam_rst_n),
 .raw_drop_count(dvp_drops),.diagnostics(input_diagnostics));
reg camera_error_sticky;
always @(posedge selected_pclk or negedge cam_rst_n)
 if(!cam_rst_n) camera_error_sticky<=0;
 else if(cam_error) camera_error_sticky<=1;
(* syn_preserve=1, syn_srlstyle="registers" *) reg[3:0] status_sync1,status_sync2;
always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n) begin status_sync1<=0;status_sync2<=0;end
 else begin
  status_sync1<={ddr_init_done,init_over_tx,cam_config_done,camera_error_sticky};
  status_sync2<=status_sync1;
 end
wire hs,vs,de;wire[15:0] vx,vy;
video_timing_1080 #(.H_ACTIVE(1280),.H_FRONT(110),.H_SYNC(40),.H_BACK(220),
 .V_ACTIVE(720),.V_FRONT(5),.V_SYNC(5),.V_BACK(20)) timing(.clk(pix_clk),.rst_n(pix_rst_n),.hs(hs),.vs(vs),.de(de),.x(vx),.y(vy));
wire boot_valid,boot_commit,boot_done,cfg_ready,cfg_commit_ready;
wire boot_ready,boot_commit_ready,uart_ready,uart_commit_ready;
wire selected_cfg_valid,selected_cfg_commit;
wire[5:0] selected_cfg_addr;wire[31:0] selected_cfg_data;
wire geometry_key_press,key_saved_valid,key_raw_active,key_busy;
wire scan_key_press,mirror_key_press,aux_valid,aux_ready;
wire[5:0] aux_addr;wire[31:0] aux_data,aux_context;
wire matrix_valid,plane_valid,mode_reject,last_publish_from_key,observation_active;
wire[1:0] requested_modes,committed_modes;
wire [5:0] boot_addr,uart_addr;wire[31:0] boot_data,uart_data;
wire uart_valid,uart_commit;
wire enhance_config_valid;
wire[2:0] enhance_config_mode;
wire detail_config_valid,detail_config_enable;
wire detail_status_core;
wire[31:0] uart_commits,uart_rejected;
video_config_boot_complete #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) boot(
 .clk(core_clk),.rst_n(core_rst_n),.valid(boot_valid),.addr(boot_addr),.data(boot_data),
 .ready(boot_ready),.commit(boot_commit),.commit_ready(boot_commit_ready),.boot_done(boot_done));
uart_config_rx_complete #(.CLK_HZ(CORE_CLK_HZ),.BAUD(115200),.REGISTER_COUNT(18)) serial_config(
 .clk(core_clk),.rst_n(core_rst_n),.uart_rx(uart_rx),.core_cfg_addr(uart_addr),.core_cfg_data(uart_data),
 .core_cfg_valid(uart_valid),.core_cfg_ready(uart_ready),.core_cfg_commit(uart_commit),
 .core_cfg_commit_ready(uart_commit_ready),.crc_error_count(),.framing_error_count(),
 .packet_error_count(),.rejected_count(uart_rejected),.write_count(),.commit_count(uart_commits),
 .enhance_valid(enhance_config_valid),.enhance_mode(enhance_config_mode),
 .detail_valid(detail_config_valid),.detail_enable(detail_config_enable),
 .aux_valid(aux_valid),.aux_addr(aux_addr),.aux_data(aux_data),.aux_ready(aux_ready));
wire shs,svs,sde,source_observation_pixel,source_raw_pixel;wire[15:0] srgb;
(* KEEP="TRUE" *) wire [31:0] captured,processed,repeated,bad_frames,underflow,overflow,skipped;
(* KEEP="TRUE" *) wire system_error,processing;wire[15:0] config_version;
wire map_building;wire[31:0] map_rebuild_cycles;
wire[31:0] map_invalid_pixels,map_domain_pixels,board_time_us,control_failures;
wire[3:0] map_reject_flags;
// V8 guard: 11..12 committed mirror/scan;13..14 requested mirror/scan;
// 15 plane valid;19 view capability;20 matrix valid;21 mode rejection;
// 22 last publication came from physical/software view replay.
// C128 bits23/24/25: software route/source-window/visual-H capabilities;
// bit26: source window in the currently published map (not display confirmation).
wire[31:0] pose_guard_state=32'h0b8f0000|{3'b0,detail_status_core,28'b0}|{5'b0,observation_active,26'b0}|
 {9'b0,last_publish_from_key,mode_reject,matrix_valid,4'b0,plane_valid,
 requested_modes,committed_modes,key_busy,key_raw_active,key_saved_valid,4'b0,map_reject_flags};
complete_key_debounce #(.STABLE_CYCLES(CORE_CLK_HZ/50)) geometry_key_debounce(
 .clk(core_clk),.rst_n(core_rst_n),.key_n(geometry_key_n),.press(geometry_key_press));
complete_key_debounce #(.STABLE_CYCLES(CORE_CLK_HZ/50)) scan_key_debounce(
 .clk(core_clk),.rst_n(core_rst_n),.key_n(scan_key_n),.press(scan_key_press));
complete_key_debounce #(.STABLE_CYCLES(CORE_CLK_HZ/50)) mirror_key_debounce(
 .clk(core_clk),.rst_n(core_rst_n),.key_n(mirror_key_n),.press(mirror_key_press));
complete_key_config_arbiter #(.UART_IDLE_CYCLES(CORE_CLK_HZ/5)) geometry_key_control(
 .clk(core_clk),.rst_n(core_rst_n),.key_press(geometry_key_press),
 .scan_press(scan_key_press),.mirror_press(mirror_key_press),
 .aux_valid(aux_valid),.aux_addr(aux_addr),.aux_data(aux_data),.aux_ready(aux_ready),
 .aux_context(aux_context),.matrix_valid(matrix_valid),.plane_valid(plane_valid),
 .requested_modes(requested_modes),.committed_modes(committed_modes),
 .mode_reject(mode_reject),.last_publish_from_key(last_publish_from_key),
 .observation_active(observation_active),
 .boot_valid(boot_valid),.boot_addr(boot_addr),.boot_data(boot_data),.boot_commit(boot_commit),.boot_done(boot_done),
 .boot_ready(boot_ready),.boot_commit_ready(boot_commit_ready),
 .uart_valid(uart_valid),.uart_addr(uart_addr),.uart_data(uart_data),.uart_commit(uart_commit),
 .uart_ready(uart_ready),.uart_commit_ready(uart_commit_ready),
 .cfg_valid(selected_cfg_valid),.cfg_addr(selected_cfg_addr),.cfg_data(selected_cfg_data),.cfg_commit(selected_cfg_commit),
 .cfg_ready(cfg_ready),.cfg_commit_ready(cfg_commit_ready),.map_building(map_building),
 // saved_valid proves a successful publication. The original system retains
 // its working map after subsequent failures; no system/display port changes.
 .map_available(key_saved_valid),.config_version(config_version),
 .saved_valid(key_saved_valid),.raw_active(key_raw_active),.key_busy(key_busy),
 .switch_count(),.timeout_count(),.failed_count(control_failures));
wire[511:0] throughput_stats;
wire latency_valid;
wire[511:0] latency_sample;
wire[31:0] latency_dropped;
wire enhance_command_busy,enhance_command_overrun,enhance_pixel_valid;
wire[2:0] enhance_pixel_mode;
wire enhance_status_valid,enhance_status_busy,enhance_status_overrun,enhance_status_core_valid;
wire[7:0] enhance_status_pixel,enhance_status_data;
reg[7:0] enhance_status_core;
latency_mailbox #(.WIDTH(3)) advanced_mode_mailbox(
 .source_clk(core_clk),.source_rst_n(core_rst_n),.source_valid(enhance_config_valid),
 .source_data(enhance_config_mode),.source_busy(enhance_command_busy),.source_overrun(enhance_command_overrun),
 .destination_clk(pix_clk),.destination_rst_n(pix_rst_n),
 .destination_valid(enhance_pixel_valid),.destination_data(enhance_pixel_mode));
latency_mailbox #(.WIDTH(8)) advanced_status_mailbox(
 .source_clk(pix_clk),.source_rst_n(pix_rst_n),.source_valid(enhance_status_valid),
 .source_data(enhance_status_pixel),.source_busy(enhance_status_busy),.source_overrun(enhance_status_overrun),
 .destination_clk(core_clk),.destination_rst_n(core_rst_n),
 .destination_valid(enhance_status_core_valid),.destination_data(enhance_status_data));
always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n) enhance_status_core<=8'b00000011;
 else if(enhance_status_core_valid) enhance_status_core<=enhance_status_data;
wire enhanced_hs,enhanced_vs,enhanced_de,enhanced_observation_pixel;
wire[15:0] enhanced_rgb;
complete_image_enhance #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.DEFAULT_MODE(0),.OBSERVATION_SUPPORT(1)) enhancement(
 .pix_clk(pix_clk),.rst_n(pix_rst_n),.hs(shs),.vs(svs),.de(sde),.rgb565(srgb),.stats_valid(!source_observation_pixel),
 .bypass_pixel(source_observation_pixel),.out_bypass(enhanced_observation_pixel),
 .config_valid(enhance_pixel_valid),.config_mode(enhance_pixel_mode),
 .out_hs(enhanced_hs),.out_vs(enhanced_vs),.out_de(enhanced_de),.out_rgb565(enhanced_rgb),
 .status_valid(enhance_status_valid),.status_data(enhance_status_pixel));
wire detailed_hs,detailed_vs,detailed_de,detail_effective_output;
wire[15:0] detailed_rgb;
c129_detail_output detail_output(
 .core_clk(core_clk),.core_rst_n(core_rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),
 .detail_config_valid(detail_config_valid),.detail_config_enable(detail_config_enable),.source_raw_pixel(source_raw_pixel),
 .enhanced_hs(enhanced_hs),.enhanced_vs(enhanced_vs),.enhanced_de(enhanced_de),.enhanced_rgb(enhanced_rgb),
 .enhanced_observation_pixel(enhanced_observation_pixel),.detailed_hs(detailed_hs),.detailed_vs(detailed_vs),.detailed_de(detailed_de),.detailed_rgb(detailed_rgb),
 .detail_effective_output(detail_effective_output),.detail_status_core(detail_status_core));
// LED6 toggles per completed result; LED7 is sticky error; LED8 toggles per capture.
assign result_led=processed[0];
assign error_led=system_error || (underflow!=0) || (overflow!=0) || cam_error;
assign capture_led=captured[0];
video_frame_system_complete #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.FISHEYE_MODEL(1),
 .CLOCK_HZ(CORE_CLK_HZ),.FINAL_OUTPUT_DELAY(1661),.TILE_WIDTH(64),.TILE_HEIGHT(16),.DETAIL_SUPPORT(1)) system_inst(
 .clk(core_clk),.rst_n(core_system_rst_n),.cam_clk(selected_pclk),.cam_rst_n(cam_rst_n),.pix_clk(pix_clk),.pix_rst_n(pix_rst_n),
 .cam_valid(cam_valid),.cam_rgb565(cam_rgb),.cam_sof(cam_sof),.cam_frame_done(cam_done),.cam_frame_drop(cam_drop),
 .video_hs(hs),.video_vs(vs),.video_de(de),.video_x(vx),.video_y(vy),
 .hdmi_hs(shs),.hdmi_vs(svs),.hdmi_de(sde),.hdmi_rgb565(srgb),.hdmi_observation(source_observation_pixel),.hdmi_raw(source_raw_pixel),
 .output_detail_enable(detail_effective_output),
 .cfg_valid(selected_cfg_valid),.cfg_addr(selected_cfg_addr),.cfg_data(selected_cfg_data),
 .cfg_commit(selected_cfg_commit),.cfg_ready(cfg_ready),.cfg_commit_ready(cfg_commit_ready),
 .axi_awaddr(axi_awaddr),.axi_awlen(axi_awlen),.axi_awvalid(axi_awvalid),.axi_awready(axi_awready),
 .axi_wdata(axi_wdata),.axi_wstrb(axi_wstrb),.axi_wready(axi_wready),.axi_wlast(axi_wusero_last),
 .axi_araddr(axi_araddr),.axi_arlen(axi_arlen),.axi_arvalid(axi_arvalid),.axi_arready(axi_arready),
 .axi_rdata(axi_rdata),.axi_rvalid(axi_rvalid),.axi_rlast(axi_rlast),
 .captured_frames(captured),.processed_frames(processed),.repeated_frames(repeated),.dropped_frames(bad_frames),
 .display_underflow_lines(underflow),.capture_overflow_frames(overflow),.camera_skipped_frames(skipped),
 .error_sticky(system_error),.processing(processing),.config_version(config_version),
 .map_building(map_building),.map_rebuild_cycles(map_rebuild_cycles),.throughput_stats(throughput_stats),
 .map_invalid_pixels(map_invalid_pixels),.map_domain_pixels(map_domain_pixels),
 .map_reject_flags(map_reject_flags),.board_time_us(board_time_us),
 .latency_valid(latency_valid),.latency_sample(latency_sample));
uart_status_latency_tx #(.CLK_HZ(CORE_CLK_HZ),.BUILD_ID(32'h0720c129),.POSE_EVENTS(1),.VIEW_EVENTS(1)) status_tx(
 .clk(core_clk),.rst_n(core_status_rst_n),.tx(uart_tx),.config_version(config_version),
 .flags({1'b0,enhance_status_core,status_sync2,system_error,map_building,processing}),.captured(captured),.processed(processed),
 .underflow(underflow),.overflow(overflow),.dropped(bad_frames),.commits(uart_commits),
 .rejected(uart_rejected),.map_cycles(map_rebuild_cycles),.raw_valid_frames(raw_valid_frames_core),
 .diagnostics(input_diagnostics),.sccb_status(sccb_status),.throughput_stats(throughput_stats),
 .board_time_us(board_time_us),.control_failures(control_failures),
 .map_invalid_pixels(map_invalid_pixels),.map_domain_pixels(map_domain_pixels),.pose_guard_state(pose_guard_state),.view_context(aux_context),
 .latency_valid(latency_valid),.latency_sample(latency_sample),.latency_dropped(latency_dropped));
always @(posedge pix_clk) begin
 r_out<={detailed_rgb[15:11],3'b0};g_out<={detailed_rgb[10:5],2'b0};b_out<={detailed_rgb[4:0],3'b0};
 hs_out<=detailed_hs;vs_out<=detailed_vs;de_out<=detailed_de;
end
reg[26:0] heartbeat;
always @(posedge core_clk) if(!core_rst_n) begin heartbeat<=0;heart_beat_led<=0;end
else if(heartbeat==CORE_CLK_HZ/2-1) begin heartbeat<=0;heart_beat_led<=~heart_beat_led;end
else heartbeat<=heartbeat+1'b1;
        DDR3_50H u_DDR3_50H (
             .ref_clk                   (sys_clk            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output
             .ddrphy_clkin              (core_clk           ),// output
             .pll_lock                  (pll_lock           ),// output

             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug
             .debug_data                (                   ),// output [135:0]
             .debug_slice_state         (                   ),// output [51:0]
             .debug_calib_ctrl          (                   ),// output [21:0]
             .ck_dly_set_bin            (                   ),// output [7:0]
             .force_ck_dly_en           (1'b0               ),// input
             .force_ck_dly_set_bin      (8'h05              ),// input [7:0]
             .dll_step                  (                   ),// output [7:0]
             .dll_lock                  (                   ),// output
             .init_read_clk_ctrl        (2'b0               ),// input [1:0]
             .init_slip_step            (4'b0               ),// input [3:0]
             .force_read_clk_ctrl       (1'b0               ),// input
             .ddrphy_gate_update_en     (1'b0               ),// input
             .update_com_val_err_flag   (                   ),// output [3:0]
             .rd_fake_stop              (1'b0               ) // input
       );


endmodule

module video_reset_sync(input wire clk,async_n,output wire rst_n);
(* syn_preserve=1, syn_srlstyle="registers" *) reg[2:0] pipe;
always @(posedge clk or negedge async_n) if(!async_n) pipe<=0;else pipe<={pipe[1:0],1'b1};
assign rst_n=pipe[2];
endmodule

// Share both synchronizing stages. The three last-stage registers sample the
// same value on the same edge, keeping the original third-edge reset release.
module video_core_reset_sync(input wire clk,async_n,
 output wire rst_n,system_rst_n,status_rst_n);
(* syn_preserve=1, syn_srlstyle="registers" *) reg[2:0] pipe;
// Preserve the parallel last stages so the three load groups can be placed apart.
(* syn_preserve=1, syn_srlstyle="registers" *) reg system_last,status_last;
always @(posedge clk or negedge async_n)
 if(!async_n) begin pipe<=0;system_last<=0;status_last<=0;end
 else begin pipe<={pipe[1:0],1'b1};system_last<=pipe[1];status_last<=pipe[1];end
assign rst_n=pipe[2];
assign system_rst_n=system_last;
assign status_rst_n=status_last;
endmodule

// Tested C129 post-enhancement output control. The detail algorithm is unchanged.
module c129_detail_output(
 input wire core_clk,core_rst_n,pix_clk,pix_rst_n,detail_config_valid,detail_config_enable,source_raw_pixel,
 input wire enhanced_hs,enhanced_vs,enhanced_de,enhanced_observation_pixel,input wire[15:0] enhanced_rgb,
 output wire detailed_hs,detailed_vs,detailed_de,output wire[15:0] detailed_rgb,
 output wire detail_effective_output,output reg detail_status_core
);
wire detail_pixel_valid,detail_pixel_enable,detail_applied_enable;
wire detail_status_core_valid,detail_status_data;
// C129: line-rate bounded detail, default OFF. Raw-result identity comes from
// the actual displayed result slot, so KEY1 cannot sharpen an old raw frame
// while the newer corrected map is already published in the core domain.
reg[3:0] raw_pixel_pipe;
always @(posedge pix_clk or negedge pix_rst_n)
 if(!pix_rst_n)raw_pixel_pipe<=4'b1111;
 else raw_pixel_pipe<={raw_pixel_pipe[2:0],source_raw_pixel};
assign detail_effective_output=detail_applied_enable&&!source_raw_pixel;
reg detail_vs_d,detail_status_armed,detail_status_pixel_valid,detail_status_pixel;
latency_mailbox #(.WIDTH(1)) detail_command_mailbox(
 .source_clk(core_clk),.source_rst_n(core_rst_n),.source_valid(detail_config_valid),.source_data(detail_config_enable),
 .source_busy(),.source_overrun(),.destination_clk(pix_clk),.destination_rst_n(pix_rst_n),
 .destination_valid(detail_pixel_valid),.destination_data(detail_pixel_enable));
detail_enhance_rgb565 #(.LINE_CLOCKS(1650)) detail(
 .pix_clk(pix_clk),.rst_n(pix_rst_n),.hs(enhanced_hs),.vs(enhanced_vs),.de(enhanced_de),.rgb565(enhanced_rgb),
 .bypass(enhanced_observation_pixel||raw_pixel_pipe[3]),.config_valid(detail_pixel_valid),.config_enable(detail_pixel_enable),
 .out_hs(detailed_hs),.out_vs(detailed_vs),.out_de(detailed_de),.out_rgb565(detailed_rgb),
 .status_valid(),.applied_enable(detail_applied_enable));
always @(posedge pix_clk or negedge pix_rst_n)
 if(!pix_rst_n)begin detail_vs_d<=0;detail_status_armed<=0;detail_status_pixel_valid<=0;detail_status_pixel<=0;end
 else begin
  detail_vs_d<=detailed_vs;detail_status_pixel_valid<=0;
  if(detailed_vs&&!detail_vs_d&&!detailed_de)detail_status_armed<=1;
  if(detailed_de&&detail_status_armed)begin
   detail_status_pixel<=detail_effective_output;detail_status_pixel_valid<=1;detail_status_armed<=0;
  end
 end
latency_mailbox #(.WIDTH(1)) detail_status_mailbox(
 .source_clk(pix_clk),.source_rst_n(pix_rst_n),.source_valid(detail_status_pixel_valid),.source_data(detail_status_pixel),
 .source_busy(),.source_overrun(),.destination_clk(core_clk),.destination_rst_n(core_rst_n),
 .destination_valid(detail_status_core_valid),.destination_data(detail_status_data));
always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n)detail_status_core<=0;
 else if(detail_status_core_valid)detail_status_core<=detail_status_data;
endmodule
