// Passive DVP observation; no signals from this block control the video path.
// A request/acknowledge mailbox holds ALL source fields until the next request.
// The core waits three extra cycles after synchronized ack before copying data.
// Source reset deliberately does not depend on camera configuration completion.
module video_input_diagnostics #(parameter SAMPLE_CYCLES=1000000)(
 input wire core_clk,core_rst_n,pclk,source_rst_n,
 input wire vsync,href,input wire[7:0] data,
 input wire raw_error,bayer_error,camera_reset_released,
 input wire[31:0] raw_drop_count,
 output wire[479:0] diagnostics
);
 reg request,ack;
 (* syn_preserve=1,syn_srlstyle="registers" *) reg req1,req2,ack1,ack2;
 (* syn_preserve=1,syn_srlstyle="registers" *) reg[1:0] levels1,levels2;
 reg[31:0] pclk_edges,vsync_rises,vsync_falls,href_rises,href_cycles;
 reg[31:0] line_pixels,last_line,min_line,max_line;
 reg[31:0] rise_lines,fall_lines,rise_pixels,fall_pixels;
 reg[31:0] last_rise_lines,last_fall_lines,last_rise_pixels,last_fall_pixels;
 reg vs_d,href_d,raw_seen,bayer_seen;
 reg[447:0] held_snapshot,core_snapshot;
 reg[31:0] sample_timer,age;
 reg busy,snapshot_valid;
 reg[2:0] settle;
 wire vr=vsync&&!vs_d,vf=!vsync&&vs_d;
 wire hr=href&&!href_d,hf=!href&&href_d;
 wire[31:0] source_flags={8'b0,data,10'b0,camera_reset_released,
                         bayer_seen|bayer_error,raw_seen|raw_error,href,vsync,1'b0};
 wire[31:0] output_flags=core_snapshot[447:416] |
                          {22'b0,levels2,1'b0,busy,5'b0,snapshot_valid};
 assign diagnostics={age,output_flags,core_snapshot[415:0]};
 always @(posedge pclk or negedge source_rst_n)
 if(!source_rst_n) begin
  req1<=0;req2<=0;ack<=0;held_snapshot<=0;
  pclk_edges<=0;vsync_rises<=0;vsync_falls<=0;href_rises<=0;href_cycles<=0;
  line_pixels<=0;last_line<=0;min_line<=32'hffffffff;max_line<=0;
  rise_lines<=0;fall_lines<=0;rise_pixels<=0;fall_pixels<=0;
  last_rise_lines<=0;last_fall_lines<=0;last_rise_pixels<=0;last_fall_pixels<=0;
  vs_d<=0;href_d<=0;raw_seen<=0;bayer_seen<=0;
 end else begin
  req1<=request;req2<=req1;vs_d<=vsync;href_d<=href;
  pclk_edges<=pclk_edges+1'b1;
  if(raw_error) raw_seen<=1;
  if(bayer_error) bayer_seen<=1;
  if(hr) href_rises<=href_rises+1'b1;
  if(href) begin
   href_cycles<=href_cycles+1'b1;
   line_pixels<=href_d?line_pixels+1'b1:32'd1;
   rise_pixels<=rise_pixels+1'b1;fall_pixels<=fall_pixels+1'b1;
  end
  if(hf) begin
   last_line<=line_pixels;
   if(line_pixels<min_line) min_line<=line_pixels;
   if(line_pixels>max_line) max_line<=line_pixels;
   rise_lines<=rise_lines+1'b1;fall_lines<=fall_lines+1'b1;
  end
  if(vr) begin
   vsync_rises<=vsync_rises+1'b1;
   last_rise_lines<=rise_lines+hf;last_rise_pixels<=rise_pixels+href;
   rise_lines<=0;rise_pixels<=0;
  end
  if(vf) begin
   vsync_falls<=vsync_falls+1'b1;
   last_fall_lines<=fall_lines+hf;last_fall_pixels<=fall_pixels+href;
   fall_lines<=0;fall_pixels<=0;
  end
  if(req2!=ack) begin
   held_snapshot<={source_flags,raw_drop_count,last_fall_pixels,last_rise_pixels,
                   last_fall_lines,last_rise_lines,max_line,
                   (min_line==32'hffffffff ? 32'd0 : min_line),last_line,
                   href_cycles,href_rises,vsync_falls,vsync_rises,pclk_edges};
   ack<=req2;
  end
 end
 always @(posedge core_clk or negedge core_rst_n)
 if(!core_rst_n) begin
  request<=0;ack1<=0;ack2<=0;levels1<=0;levels2<=0;
  sample_timer<=0;age<=0;busy<=0;snapshot_valid<=0;settle<=0;core_snapshot<=0;
 end else begin
  ack1<=ack;ack2<=ack1;levels1<={href,vsync};levels2<=levels1;
  if(age!=32'hffffffff) age<=age+1'b1;
  if(!busy) begin
   if(sample_timer==SAMPLE_CYCLES-1) begin
    request<=~request;busy<=1;sample_timer<=0;settle<=0;
   end else sample_timer<=sample_timer+1'b1;
  end else if(ack2==request) begin
   if(settle==3) begin
    core_snapshot<=held_snapshot;age<=0;busy<=0;snapshot_valid<=1;settle<=0;
   end else settle<=settle+1'b1;
  end
 end
endmodule
