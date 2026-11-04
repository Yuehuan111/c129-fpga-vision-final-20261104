// Tiled map-driven warp engine. Map entries are 64 bits, low 39 bits are
// {inside,xQ11.8,yQ11.8}. Memory addresses are bytes. One external memory
// transaction is outstanding; map/source prefetch overlaps pixel rendering.
// Unsupported bounding rectangles raise error; they are never clipped silently.
module fast_warp_engine #(
 parameter IMG_WIDTH=1920,IMG_HEIGHT=1080,
 parameter TILE_WIDTH=128,TILE_HEIGHT=16,
 parameter MAX_ROI_WIDTH=256,MAX_ROI_HEIGHT=64
)(
 input wire clk,rst_n,start,output wire busy,output reg done,error,
 input wire[31:0] source_base,output_base,map_base,
 input wire signed[31:0] gain_q8,offset,
 output wire cmd_valid,input wire cmd_ready,output wire cmd_write,
 output wire[31:0] cmd_addr,output wire[3:0] cmd_len,
 output wire w_valid,input wire w_ready,output wire[255:0] w_data,output wire[31:0] w_strb,
 input wire r_valid,output wire r_ready,input wire[255:0] r_data,input wire r_last,
 input wire mem_done,mem_error,
 output reg[31:0] cycles,tiles_completed,map_read_beats,source_read_beats
);
 localparam TILE_COLUMNS=(IMG_WIDTH+TILE_WIDTH-1)/TILE_WIDTH;
 localparam TILE_ROWS=(IMG_HEIGHT+TILE_HEIGHT-1)/TILE_HEIGHT;
 localparam TILE_COUNT=TILE_COLUMNS*TILE_ROWS;
 localparam MAP_ROW_WORDS=TILE_WIDTH/4,MAP_DEPTH=MAP_ROW_WORDS*TILE_HEIGHT;
 localparam SOURCE_ROW_WORDS=MAX_ROI_WIDTH/16;
 localparam SOURCE_DEPTH=SOURCE_ROW_WORDS*((MAX_ROI_HEIGHT+1)/2);
 localparam SOURCE_ADDR_BITS=$clog2(SOURCE_DEPTH);
 localparam OUT_ROW_WORDS=TILE_WIDTH/16,OUT_DEPTH=OUT_ROW_WORDS*TILE_HEIGHT;
 localparam P_IDLE=0,P_MAP_CMD=1,P_MAP_DATA=2,P_BOUNDS=3,P_SOURCE_CMD=4,P_SOURCE_DATA=5;
 localparam R_IDLE=0,R_RUN=1,R_FLUSH=2;
 localparam F_IDLE=0,F_CMD=1,F_DATA=2,F_WAIT=3,F_NEXT=4;
 reg active;
 assign busy=active;
 reg[31:0] frame_source,frame_output,frame_map;
 reg signed[31:0] frame_gain,frame_offset;
 reg[2:0] pf_state,flush_state;
 reg[1:0] render_state,buffer_in_use,buffer_ready,output_ready;
 reg[31:0] pf_sequence,render_sequence,flush_sequence;
 reg pf_bank,render_bank,flush_bank;
 reg[15:0] next_tile_x,next_tile_y;
 reg[15:0] tile_x[0:1],tile_y[0:1],tile_w[0:1],tile_h[0:1];
 reg[15:0] roi_x[0:1],roi_y[0:1],roi_w[0:1],roi_h[0:1];
 reg[15:0] pf_row,pf_beat,pf_chunk,min_x,min_y,max_x,max_y;
 reg any_inside;
 reg[15:0] issue_x,issue_y,flush_row,flush_beat,flush_chunk;
 reg issued_all;
 reg[3:0] output_lane;
 reg[255:0] output_pack,flush_data;
 // Four map banks accept four entries per DDR beat and supply one per cycle.
 reg[38:0] map0[0:2*MAP_DEPTH-1],map1[0:2*MAP_DEPTH-1];
 reg[38:0] map2[0:2*MAP_DEPTH-1],map3[0:2*MAP_DEPTH-1];
 // Each source bank stores eight same-parity pixels per 128-bit word.
 // An entire incoming beat writes one even-x bank and one odd-x bank.
 reg[127:0] source00[0:2*SOURCE_DEPTH-1],source01[0:2*SOURCE_DEPTH-1];
 reg[127:0] source10[0:2*SOURCE_DEPTH-1],source11[0:2*SOURCE_DEPTH-1];
 // Independent output banks permit rendering tile N+1 while tile N is
 // physically written. A tile's source/map/output bank is released only by
 // that tile's final DDR completion, never by interpolation completion.
 reg[255:0] output_words[0:2*OUT_DEPTH-1];
 reg bus_occupied,bus_write_owner,command_locked,locked_write,abort_pending;
 wire pf_command=(pf_state==P_MAP_CMD)||(pf_state==P_SOURCE_CMD);
 wire flush_command=flush_state==F_CMD;
 wire select_flush=command_locked?locked_write:flush_command;
 wire pf_command_ready=active&&!abort_pending&&!bus_occupied&&!select_flush&&cmd_ready;
 wire flush_command_ready=active&&!abort_pending&&!bus_occupied&&select_flush&&cmd_ready;
 wire pf_done=bus_occupied&&!bus_write_owner&&mem_done;
 wire flush_done=bus_occupied&&bus_write_owner&&mem_done;
 wire pf_read=bus_occupied&&!bus_write_owner&&r_valid&&r_ready;
 wire pf_is_map=(pf_state==P_MAP_CMD)||(pf_state==P_MAP_DATA);
 wire[15:0] pf_row_beats=pf_is_map?(tile_w[pf_bank]/4):(roi_w[pf_bank]/16);
 wire[15:0] pf_remaining=pf_row_beats-pf_chunk;
 wire[4:0] pf_burst_beats=(pf_remaining>16)?5'd16:pf_remaining[4:0];
 wire[15:0] flush_remaining=tile_w[flush_bank]/16-flush_chunk;
 wire[4:0] flush_burst_beats=(flush_remaining>16)?5'd16:flush_remaining[4:0];
 wire flush_row_end=flush_chunk+flush_burst_beats>=tile_w[flush_bank]/16;
 wire flush_finish=(flush_state==F_NEXT)&&flush_row_end&&(flush_row+1>=tile_h[flush_bank]);
 assign cmd_valid=active&&!abort_pending&&!bus_occupied&&(flush_command||pf_command);
 assign cmd_write=select_flush;
 assign cmd_addr=select_flush?
  frame_output+(((tile_y[flush_bank]+flush_row)*IMG_WIDTH+tile_x[flush_bank])<<1)+(flush_chunk<<5):
  (pf_state==P_MAP_CMD?
   frame_map+(((tile_y[pf_bank]+pf_row)*IMG_WIDTH+tile_x[pf_bank])<<3)+(pf_chunk<<5):
   frame_source+(((roi_y[pf_bank]+pf_row)*IMG_WIDTH+roi_x[pf_bank])<<1)+(pf_chunk<<5));
 assign cmd_len=select_flush?(flush_burst_beats-1'b1):(pf_burst_beats-1'b1);
 assign w_valid=active&&bus_occupied&&bus_write_owner&&(flush_state==F_DATA);
 assign w_data=flush_data;assign w_strb=32'hffffffff;
 assign r_ready=active&&bus_occupied&&!bus_write_owner&&
  ((pf_state==P_MAP_DATA)||(pf_state==P_SOURCE_DATA));

 // Two front-end pipeline stages: map RAM read, then four source-bank reads.
 reg front_valid0,front_valid1;
 reg[38:0] map_q;
 reg[15:0] out_x0,out_y0,out_x1,out_y1;
 reg last0,last1,inside1,x_parity1,y_parity1,duplicate_x1,duplicate_y1;
 reg[7:0] fx1,fy1;reg[2:0] even_lane1,odd_lane1;
 reg[127:0] bank00_q,bank01_q,bank10_q,bank11_q;
 wire[15:0] local_x={5'b0,map_q[37:27]}-roi_x[render_bank];
 wire[15:0] local_y={5'b0,map_q[18:8]}-roi_y[render_bank];
 wire[15:0] even_x=local_x[0]?local_x+1'b1:local_x;
 wire[15:0] odd_x=local_x[0]?local_x:local_x+1'b1;
 wire[15:0] even_y=local_y[0]?local_y+1'b1:local_y;
 wire[15:0] odd_y=local_y[0]?local_y:local_y+1'b1;
 wire[SOURCE_ADDR_BITS-1:0] addr00=(even_y>>1)*SOURCE_ROW_WORDS+(even_x>>4);
 wire[SOURCE_ADDR_BITS-1:0] addr01=(even_y>>1)*SOURCE_ROW_WORDS+(odd_x>>4);
 wire[SOURCE_ADDR_BITS-1:0] addr10=(odd_y>>1)*SOURCE_ROW_WORDS+(even_x>>4);
 wire[SOURCE_ADDR_BITS-1:0] addr11=(odd_y>>1)*SOURCE_ROW_WORDS+(odd_x>>4);
 wire[15:0] p_even_even=bank00_q[even_lane1*16+:16];
 wire[15:0] p_even_odd=bank01_q[odd_lane1*16+:16];
 wire[15:0] p_odd_even=bank10_q[even_lane1*16+:16];
 wire[15:0] p_odd_odd=bank11_q[odd_lane1*16+:16];
 wire[15:0] p00=y_parity1?(x_parity1?p_odd_odd:p_odd_even):(x_parity1?p_even_odd:p_even_even);
 wire[15:0] p10_raw=y_parity1?(x_parity1?p_odd_even:p_odd_odd):(x_parity1?p_even_even:p_even_odd);
 wire[15:0] p01_raw=y_parity1?(x_parity1?p_even_odd:p_even_even):(x_parity1?p_odd_odd:p_odd_even);
 wire[15:0] p11_raw=y_parity1?(x_parity1?p_even_even:p_even_odd):(x_parity1?p_odd_even:p_odd_odd);
 wire[15:0] p10=duplicate_x1?p00:p10_raw;
 wire[15:0] p01=duplicate_y1?p00:p01_raw;
 wire[15:0] p11=duplicate_x1?(duplicate_y1?p00:p01_raw):(duplicate_y1?p10_raw:p11_raw);
 wire interp_ready,interpolated_valid,interpolated_last;
 wire[15:0] interpolated_pixel,interpolated_x,interpolated_y;
 wire interpolated_ready=active&&(render_state==R_RUN);
 wire issue_pixel=active&&(render_state==R_RUN)&&!issued_all;
 bilinear_rgb565_pipeline interpolation(
  .clk(clk),.rst_n(rst_n&&active),.s_valid(front_valid1),.s_ready(interp_ready),
  .s_pixels({p11,p01,p10,p00}),.s_fx(fx1),.s_fy(fy1),.s_inside(inside1),
  .s_x(out_x1),.s_y(out_y1),.s_last(last1),.s_gain_q8(frame_gain),.s_offset(frame_offset),
  .m_valid(interpolated_valid),.m_ready(interpolated_ready),.m_rgb565(interpolated_pixel),
  .m_x(interpolated_x),.m_y(interpolated_y),.m_last(interpolated_last));
 integer i,j,map_index,source_index,output_index;
 reg[127:0] unpack_even,unpack_odd;
 reg[63:0] entry_temp;
 reg[15:0] temp_min_x,temp_min_y,temp_max_x,temp_max_y,entry_x,entry_y,entry_right,entry_bottom;
 reg temp_seen;
 reg[15:0] aligned_left,aligned_width,bounded_height;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   active<=0;done<=0;error<=0;cycles<=0;tiles_completed<=0;map_read_beats<=0;source_read_beats<=0;
   frame_source<=0;frame_output<=0;frame_map<=0;frame_gain<=0;frame_offset<=0;
   pf_state<=P_IDLE;render_state<=R_IDLE;flush_state<=F_IDLE;buffer_in_use<=0;buffer_ready<=0;output_ready<=0;
   pf_sequence<=0;render_sequence<=0;flush_sequence<=0;pf_bank<=0;render_bank<=0;flush_bank<=0;next_tile_x<=0;next_tile_y<=0;
   pf_row<=0;pf_beat<=0;pf_chunk<=0;min_x<=0;min_y<=0;max_x<=0;max_y<=0;any_inside<=0;
   issue_x<=0;issue_y<=0;flush_row<=0;flush_beat<=0;flush_chunk<=0;issued_all<=0;output_lane<=0;output_pack<=0;flush_data<=0;
   bus_occupied<=0;bus_write_owner<=0;command_locked<=0;locked_write<=0;abort_pending<=0;
   front_valid0<=0;front_valid1<=0;map_q<=0;out_x0<=0;out_y0<=0;out_x1<=0;out_y1<=0;
   last0<=0;last1<=0;inside1<=0;x_parity1<=0;y_parity1<=0;duplicate_x1<=0;duplicate_y1<=0;
   fx1<=0;fy1<=0;even_lane1<=0;odd_lane1<=0;bank00_q<=0;bank01_q<=0;bank10_q<=0;bank11_q<=0;
   for(i=0;i<2;i=i+1)begin tile_x[i]<=0;tile_y[i]<=0;tile_w[i]<=0;tile_h[i]<=0;roi_x[i]<=0;roi_y[i]<=0;roi_w[i]<=0;roi_h[i]<=0;end
  end else begin
   done<=0;
   if(cmd_valid&&!cmd_ready)begin command_locked<=1;locked_write<=cmd_write;end
   if(cmd_valid&&cmd_ready)begin bus_occupied<=1;bus_write_owner<=cmd_write;command_locked<=0;end
   if(mem_done&&bus_occupied)bus_occupied<=0;
   if(start&&!active&&!bus_occupied)begin
    active<=1;error<=0;abort_pending<=0;command_locked<=0;cycles<=0;tiles_completed<=0;map_read_beats<=0;source_read_beats<=0;
    frame_source<=source_base;frame_output<=output_base;frame_map<=map_base;frame_gain<=gain_q8;frame_offset<=offset;
    pf_state<=P_IDLE;render_state<=R_IDLE;flush_state<=F_IDLE;buffer_in_use<=0;buffer_ready<=0;output_ready<=0;
    pf_sequence<=0;render_sequence<=0;flush_sequence<=0;next_tile_x<=0;next_tile_y<=0;front_valid0<=0;front_valid1<=0;
    if(IMG_WIDTH<16||IMG_WIDTH>2048||IMG_HEIGHT<1||IMG_HEIGHT>2048||(IMG_WIDTH%16)!=0||
       TILE_WIDTH<16||TILE_WIDTH>256||(TILE_WIDTH&(TILE_WIDTH-1))!=0||TILE_HEIGHT!=16||
       MAX_ROI_WIDTH<16||MAX_ROI_WIDTH>512||(MAX_ROI_WIDTH&(MAX_ROI_WIDTH-1))!=0||MAX_ROI_HEIGHT!=64)begin
     active<=0;error<=1;
    end
   end else if(active)begin
    cycles<=cycles+1'b1;
    // Loader allocates buffers in raster order; a buffer is not released until
    // its output tile has completed its final physical DDR write.
    case(pf_state)
     P_IDLE: if(pf_sequence<TILE_COUNT&&!buffer_in_use[pf_sequence[0]])begin
      pf_bank<=pf_sequence[0];buffer_in_use[pf_sequence[0]]<=1;
      tile_x[pf_sequence[0]]<=next_tile_x;tile_y[pf_sequence[0]]<=next_tile_y;
      tile_w[pf_sequence[0]]<=(next_tile_x+TILE_WIDTH>IMG_WIDTH)?IMG_WIDTH-next_tile_x:TILE_WIDTH;
      tile_h[pf_sequence[0]]<=(next_tile_y+TILE_HEIGHT>IMG_HEIGHT)?IMG_HEIGHT-next_tile_y:TILE_HEIGHT;
      if(next_tile_x+TILE_WIDTH>=IMG_WIDTH)begin next_tile_x<=0;next_tile_y<=next_tile_y+TILE_HEIGHT;end
      else next_tile_x<=next_tile_x+TILE_WIDTH;
      min_x<=IMG_WIDTH-1;min_y<=IMG_HEIGHT-1;max_x<=0;max_y<=0;any_inside<=0;
      pf_row<=0;pf_beat<=0;pf_chunk<=0;pf_state<=P_MAP_CMD;
     end
     P_MAP_CMD: if(pf_command_ready)begin pf_beat<=0;pf_state<=P_MAP_DATA;end
     P_MAP_DATA: begin
      if(pf_read)begin
       map_read_beats<=map_read_beats+1'b1;
       map_index=pf_bank*MAP_DEPTH+pf_row*MAP_ROW_WORDS+pf_chunk+pf_beat;
       map0[map_index]<=r_data[38:0];map1[map_index]<=r_data[102:64];
       map2[map_index]<=r_data[166:128];map3[map_index]<=r_data[230:192];
       temp_min_x=min_x;temp_min_y=min_y;temp_max_x=max_x;temp_max_y=max_y;temp_seen=any_inside;
       for(j=0;j<4;j=j+1)begin
        entry_temp=r_data[j*64+:64];entry_x={5'b0,entry_temp[37:27]};entry_y={5'b0,entry_temp[18:8]};
        if(entry_temp[38])begin
         if(entry_x>=IMG_WIDTH||entry_y>=IMG_HEIGHT)begin error<=1;abort_pending<=1;end
         entry_right=(entry_x<IMG_WIDTH-1&&entry_temp[26:19]!=0)?entry_x+1'b1:entry_x;
         entry_bottom=(entry_y<IMG_HEIGHT-1&&entry_temp[7:0]!=0)?entry_y+1'b1:entry_y;
         if(!temp_seen||entry_x<temp_min_x)temp_min_x=entry_x;
         if(!temp_seen||entry_y<temp_min_y)temp_min_y=entry_y;
         if(!temp_seen||entry_right>temp_max_x)temp_max_x=entry_right;
         if(!temp_seen||entry_bottom>temp_max_y)temp_max_y=entry_bottom;
         temp_seen=1;
        end
       end
       min_x<=temp_min_x;min_y<=temp_min_y;max_x<=temp_max_x;max_y<=temp_max_y;any_inside<=temp_seen;
       if(r_last!==(pf_beat+1==pf_burst_beats))begin error<=1;abort_pending<=1;end
       pf_beat<=pf_beat+1'b1;
      end
      if(pf_done)begin
       if(pf_chunk+pf_burst_beats<pf_row_beats)begin pf_chunk<=pf_chunk+pf_burst_beats;pf_state<=P_MAP_CMD;end
       else begin
        pf_chunk<=0;
        if(pf_row+1==tile_h[pf_bank])pf_state<=P_BOUNDS;
        else begin pf_row<=pf_row+1'b1;pf_state<=P_MAP_CMD;end
       end
      end
     end
     P_BOUNDS: begin
      aligned_left={min_x[15:4],4'b0};aligned_width=({max_x[15:4],4'b0}-aligned_left)+16;
      bounded_height=max_y-min_y+1'b1;
      if(any_inside&&(aligned_width>MAX_ROI_WIDTH||bounded_height>MAX_ROI_HEIGHT))begin error<=1;active<=0;end
      else begin
       roi_x[pf_bank]<=aligned_left;roi_y[pf_bank]<=min_y;
       roi_w[pf_bank]<=any_inside?aligned_width:0;roi_h[pf_bank]<=any_inside?bounded_height:0;
       if(!any_inside)begin buffer_ready[pf_bank]<=1;pf_sequence<=pf_sequence+1'b1;pf_state<=P_IDLE;end
       else begin pf_row<=0;pf_beat<=0;pf_chunk<=0;pf_state<=P_SOURCE_CMD;end
      end
     end
     P_SOURCE_CMD: if(pf_command_ready)begin pf_beat<=0;pf_state<=P_SOURCE_DATA;end
     P_SOURCE_DATA: begin
      if(pf_read)begin
       source_read_beats<=source_read_beats+1'b1;
       source_index=pf_bank*SOURCE_DEPTH+(pf_row>>1)*SOURCE_ROW_WORDS+pf_chunk+pf_beat;
       for(j=0;j<8;j=j+1)begin unpack_even[j*16+:16]=r_data[j*32+:16];unpack_odd[j*16+:16]=r_data[j*32+16+:16];end
       if(pf_row[0])begin source10[source_index]<=unpack_even;source11[source_index]<=unpack_odd;end
       else begin source00[source_index]<=unpack_even;source01[source_index]<=unpack_odd;end
       if(r_last!==(pf_beat+1==pf_burst_beats))begin error<=1;abort_pending<=1;end
       pf_beat<=pf_beat+1'b1;
      end
      if(pf_done)begin
       if(pf_chunk+pf_burst_beats<pf_row_beats)begin pf_chunk<=pf_chunk+pf_burst_beats;pf_state<=P_SOURCE_CMD;end
       else begin
        pf_chunk<=0;
        if(pf_row+1==roi_h[pf_bank])begin buffer_ready[pf_bank]<=1;pf_sequence<=pf_sequence+1'b1;pf_state<=P_IDLE;end
        else begin pf_row<=pf_row+1'b1;pf_state<=P_SOURCE_CMD;end
       end
      end
     end
     default: begin error<=1;active<=0;end
    endcase

    case(render_state)
     R_IDLE: if(render_sequence<TILE_COUNT&&buffer_ready[render_sequence[0]])begin
      render_bank<=render_sequence[0];buffer_ready[render_sequence[0]]<=0;
      issue_x<=0;issue_y<=0;issued_all<=0;output_lane<=0;output_pack<=0;render_state<=R_RUN;
     end
     R_RUN: begin
      if(issue_pixel&&interp_ready)begin
       if(issue_x+1==tile_w[render_bank])begin
        issue_x<=0;
        if(issue_y+1==tile_h[render_bank])issued_all<=1;else issue_y<=issue_y+1'b1;
       end else issue_x<=issue_x+1'b1;
      end
      if(interpolated_valid&&interpolated_ready)begin
       output_pack[output_lane*16+:16]<=interpolated_pixel;
       if(output_lane==15)begin
        output_index=render_bank*OUT_DEPTH+(interpolated_y-tile_y[render_bank])*OUT_ROW_WORDS+((interpolated_x-tile_x[render_bank])>>4);
        output_words[output_index]<={interpolated_pixel,output_pack[239:0]};output_lane<=0;
       end else output_lane<=output_lane+1'b1;
       if(interpolated_last)begin
        if(output_lane!=15)begin error<=1;active<=0;end
        output_ready[render_bank]<=1;render_sequence<=render_sequence+1'b1;render_state<=R_IDLE;
       end
      end
     end
     default: begin error<=1;active<=0;end
    endcase
    if(interp_ready)begin
     front_valid0<=issue_pixel;front_valid1<=front_valid0;
     if(issue_pixel)begin
      map_index=render_bank*MAP_DEPTH+issue_y*MAP_ROW_WORDS+(issue_x>>2);
      case(issue_x[1:0])
       0:map_q<=map0[map_index];1:map_q<=map1[map_index];2:map_q<=map2[map_index];3:map_q<=map3[map_index];
      endcase
      out_x0<=tile_x[render_bank]+issue_x;out_y0<=tile_y[render_bank]+issue_y;
      last0<=issue_x+1==tile_w[render_bank]&&issue_y+1==tile_h[render_bank];
     end
     bank00_q<=source00[render_bank*SOURCE_DEPTH+addr00];bank01_q<=source01[render_bank*SOURCE_DEPTH+addr01];
     bank10_q<=source10[render_bank*SOURCE_DEPTH+addr10];bank11_q<=source11[render_bank*SOURCE_DEPTH+addr11];
     even_lane1<=even_x[3:1];odd_lane1<=odd_x[3:1];x_parity1<=local_x[0];y_parity1<=local_y[0];
     duplicate_x1<=map_q[37:27]==IMG_WIDTH-1||map_q[26:19]==0;
     duplicate_y1<=map_q[18:8]==IMG_HEIGHT-1||map_q[7:0]==0;
     inside1<=map_q[38];fx1<=map_q[26:19];fy1<=map_q[7:0];out_x1<=out_x0;out_y1<=out_y0;last1<=last0;
    end
    case(flush_state)
     F_IDLE: if(flush_sequence<TILE_COUNT&&output_ready[flush_sequence[0]])begin
      flush_bank<=flush_sequence[0];flush_row<=0;flush_beat<=0;flush_chunk<=0;flush_state<=F_CMD;
     end
     F_CMD: if(flush_command_ready)begin
      flush_beat<=0;flush_data<=output_words[flush_bank*OUT_DEPTH+flush_row*OUT_ROW_WORDS+flush_chunk];flush_state<=F_DATA;
     end
     F_DATA: begin
      if(w_valid&&w_ready)begin
       if(flush_beat+1==flush_burst_beats)flush_state<=F_WAIT;
       else begin flush_beat<=flush_beat+1'b1;flush_data<=output_words[flush_bank*OUT_DEPTH+flush_row*OUT_ROW_WORDS+flush_chunk+flush_beat+1];end
      end
      if(flush_done)flush_state<=F_NEXT;
     end
     F_WAIT: if(flush_done)flush_state<=F_NEXT;
     F_NEXT: if(flush_finish)begin
      flush_state<=F_IDLE;output_ready[flush_bank]<=0;buffer_in_use[flush_bank]<=0;
      tiles_completed<=tiles_completed+1'b1;flush_sequence<=flush_sequence+1'b1;
      if(flush_sequence+1==TILE_COUNT)begin done<=1;active<=0;end
     end
      else if(!flush_row_end)begin flush_chunk<=flush_chunk+flush_burst_beats;flush_state<=F_CMD;end
      else begin flush_row<=flush_row+1'b1;flush_chunk<=0;flush_state<=F_CMD;end
     default: begin error<=1;active<=0;end
    endcase
    if(mem_error&&bus_occupied)begin error<=1;abort_pending<=1;end
    // Drain the accepted burst before aborting, so the shared bridge can serve
    // other clients and a later start can recover without a board-wide reset.
    if(abort_pending&&!bus_occupied)begin active<=0;done<=0;end
   end
  end
 end
endmodule
