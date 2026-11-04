// C128: small coefficient transactions, not host-generated image maps.
// mode={scan,mirror}; 00 preserves all original H words without requantization.
module complete_key_config_arbiter #(parameter integer UART_IDLE_CYCLES=20000000)(
 input wire clk,rst_n,key_press,scan_press,mirror_press,
 input wire boot_valid,input wire[5:0] boot_addr,input wire[31:0] boot_data,
 input wire boot_commit,boot_done,output reg boot_ready,boot_commit_ready,
 input wire uart_valid,input wire[5:0] uart_addr,input wire[31:0] uart_data,
 input wire uart_commit,output reg uart_ready,uart_commit_ready,
 input wire aux_valid,input wire[5:0] aux_addr,input wire[31:0] aux_data,output reg aux_ready,
 output reg cfg_valid,output reg[5:0] cfg_addr,output reg[31:0] cfg_data,
 output reg cfg_commit,input wire cfg_ready,cfg_commit_ready,
 input wire map_building,map_available,input wire[15:0] config_version,
 output reg saved_valid,raw_active,output wire key_busy,
 output reg[31:0] switch_count,timeout_count,failed_count,aux_context,
 output reg matrix_valid,plane_valid,output reg[1:0] requested_modes,committed_modes,
 output reg mode_reject,last_publish_from_key,observation_active
);
 localparam OWNER_BOOT=0,OWNER_NONE=1,OWNER_UART=2,OWNER_KEY=3;
 localparam RECEIVE=0,COPY_READ=1,COPY_WAIT=2,COPY_WRITE=3,PREPARE=4,
  COMPOSE_START=5,COMPOSE_WAIT=6,OUT_BEGIN=7,OUT_READ=8,OUT_WAIT=9,
  OUT_WORD=10,OUT_COMMIT=11,WAIT_PUBLISH=12,SEND_ABORT=13;
 function integer bits_for;input integer value;integer n;
 begin bits_for=1;for(n=value-1;n>1;n=n>>1)bits_for=bits_for+1;end endfunction
 localparam IDLE_BITS=bits_for(UART_IDLE_CYCLES);
 reg[3:0] state;
 reg[1:0] owner,transaction_modes;
 reg saved_bank,work_bank,transaction_raw,transaction_open,tag_good,mode_good;
 reg[4:0] expected_index,copy_index,copy_limit,out_index;
 reg[31:0] candidate_mode,out_word;
 reg[15:0] issued_version;
 reg building_seen,mode_dirty,basis_changed,basis_pending;
 reg requested_raw,requested_observation,route_pending,transaction_observation;
 reg[31:0] basis_word;
 reg[IDLE_BITS-1:0] idle_count,aux_idle_count;
 // Synchronous single-word memory ports; no RAM reset or parallel payload copy.
 (* syn_ramstyle="block_ram" *) reg[31:0] base_mem[0:63];
 (* syn_ramstyle="block_ram" *) reg[31:0] factors[0:127];
 (* syn_ramstyle="block_ram" *) reg[31:0] composed[0:15];
 reg[31:0] base_q,factor_q,composed_q;
 wire[3:0] composed_read_address=out_index[3:0]-4'd2;
 reg factor_bank,transaction_factor_bank;
 reg aux_open,aux_flags_written,aux_scan_valid,aux_bad;
 reg[5:0] aux_index;
 reg[3:0] aux_nonzero;
 reg[31:0] aux_candidate_context;
 wire input_valid=owner==OWNER_BOOT?boot_valid:uart_valid;
 wire[5:0] input_addr=owner==OWNER_BOOT?boot_addr:uart_addr;
 wire[31:0] input_data=owner==OWNER_BOOT?boot_data:uart_data;
 wire input_ready=owner==OWNER_BOOT?boot_ready:uart_ready;
 wire input_accept=input_valid&&input_ready;
 wire input_commit=owner==OWNER_BOOT?boot_commit:uart_commit;
 wire input_commit_ready=owner==OWNER_BOOT?boot_commit_ready:uart_commit_ready;
 wire[5:0] expected_addr=expected_index<18?{1'b0,expected_index}:{1'b0,expected_index}+6'd14;
 wire accepted_parameter=input_accept&&transaction_open&&expected_index<27&&input_addr==expected_addr;
 wire aux_accept=aux_valid&&aux_ready;
 wire aux_timeout_now=aux_open&&!aux_accept&&aux_idle_count>=UART_IDLE_CYCLES-1;
 wire basis_publish_now=state==WAIT_PUBLISH&&config_version!=issued_version&&basis_changed;
 wire aux_coefficient=aux_accept&&aux_open&&aux_flags_written&&aux_index<36&&aux_addr==aux_index;
 // FC/VI is an explicitly visual H transaction. It cannot compose a new
 // visual inverse H with the old static plane/mirror factors accidentally.
 wire visual_request=input_data[31:16]==16'h5649;
 wire pose_allowed=saved_valid&&!raw_active&&map_available&&!map_building&&
                   input_data[15:0]==config_version&&
                   (input_data[31:16]==16'h504f||
                    (visual_request&&observation_active&&requested_modes==0&&committed_modes==0));
 wire[3:0] compose_h_index,compose_c_index,compose_result_index;
 wire compose_valid,compose_done,compose_error;
 wire[31:0] compose_result;
 wire[5:0] factor_offset=transaction_modes==1?6'd9:transaction_modes==2?6'd18:6'd27;
 reg[5:0] base_read_address;
 wire[6:0] factor_read_address={transaction_factor_bank,6'b0}+factor_offset+compose_c_index;
 reg[31:0] copy_word;
 always @* begin
  base_read_address={work_bank,5'b0}+out_index;
  if(state==COPY_READ||state==COPY_WAIT||state==COPY_WRITE)base_read_address={saved_bank,5'b0}+copy_index;
  else if(state==COMPOSE_START||state==COMPOSE_WAIT)base_read_address={work_bank,5'b0}+6'd18+compose_h_index;
  else if(state==RECEIVE&&accepted_parameter&&expected_index<14)base_read_address={saved_bank,5'b0}+expected_index;
  copy_word=base_q;
  if(owner==OWNER_KEY&&transaction_raw)begin
   if(copy_index==14)copy_word=256;
   else if(copy_index==15||copy_index==16)copy_word=0;
   else if(copy_index>=18)copy_word=(copy_index==18||copy_index==22||copy_index==26)?65536:0;
  end
 end
 always @(posedge clk)begin
  base_q<=base_mem[base_read_address];factor_q<=factors[factor_read_address];
  composed_q<=composed[composed_read_address];
  if(accepted_parameter)base_mem[{work_bank,5'b0}+expected_index]<=input_data;
  else if(state==COPY_WRITE)base_mem[{work_bank,5'b0}+copy_index]<=copy_word;
  if(aux_coefficient)factors[{!factor_bank,6'b0}+aux_index]<=aux_data;
  if(compose_valid)composed[compose_result_index]<=compose_result;
 end
 complete_matrix_compose_serial matrix_composer(
  .clk(clk),.rst_n(rst_n),.start(state==COMPOSE_START),
  .h_index(compose_h_index),.c_index(compose_c_index),.h_word(base_q),.c_word(factor_q),
  .result_valid(compose_valid),.result_index(compose_result_index),.result_word(compose_result),
  .done(compose_done),.error(compose_error));
 assign key_busy=state!=RECEIVE||owner!=OWNER_NONE||aux_open||map_building||!saved_valid||!boot_done||route_pending||(!raw_active&&mode_dirty);
 function[31:0] increment_saturated;input[31:0] value;
 begin increment_saturated=&value?value:value+1'b1;end endfunction
 // E8 starts only from committed/requested 00. A failed replacement must not
 // apply a request made during that upload using the old profile afterwards.
 task reject_aux_transaction;
 begin
  aux_open<=0;mode_reject<=1;failed_count<=increment_saturated(failed_count);
  if(aux_open)begin requested_modes<=0;mode_dirty<=0;end
 end endtask
 wire mode_request_valid=aux_data[31:2]==0&&(!aux_data[1]||plane_valid)&&(aux_data[1:0]==0||matrix_valid);
 always @* begin
  cfg_valid=0;cfg_addr=0;cfg_data=0;cfg_commit=0;boot_ready=0;boot_commit_ready=0;
  uart_ready=0;uart_commit_ready=0;aux_ready=0;
  if(rst_n)begin
   // 3A is an internal receive-error abort, never a host wire command.
   if(aux_addr==6'h3a||aux_addr==6'h3e||aux_addr==6'h3f)aux_ready=1;
   else if(state==RECEIVE&&owner==OWNER_NONE&&!map_building)aux_ready=1;
   if(state==RECEIVE&&!aux_open&&!map_building)begin
    if(owner==OWNER_BOOT)begin boot_ready=1;boot_commit_ready=transaction_open&&expected_index==27&&tag_good&&mode_good;end
    else if(owner==OWNER_NONE||owner==OWNER_UART)begin
     uart_ready=1;uart_commit_ready=owner==OWNER_UART&&transaction_open&&expected_index==27&&tag_good&&mode_good;
    end
   end
   if(state==OUT_BEGIN||state==SEND_ABORT)begin cfg_valid=1;cfg_addr=6'h3f;cfg_data=state==OUT_BEGIN?32'h434f4d50:0;end
   else if(state==OUT_WORD)begin cfg_valid=1;cfg_addr=out_index<18?{1'b0,out_index}:{1'b0,out_index}+6'd14;cfg_data=out_word;end
   else if(state==OUT_COMMIT)cfg_commit=1;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=RECEIVE;owner<=OWNER_BOOT;saved_bank<=0;work_bank<=1;
   transaction_modes<=0;transaction_factor_bank<=0;transaction_raw<=0;
   requested_raw<=0;requested_observation<=0;route_pending<=0;transaction_observation<=0;observation_active<=0;
   transaction_open<=0;tag_good<=0;mode_good<=0;expected_index<=0;
   copy_index<=0;copy_limit<=0;out_index<=0;out_word<=0;candidate_mode<=1;
   issued_version<=0;building_seen<=0;mode_dirty<=0;basis_changed<=0;basis_pending<=0;basis_word<=0;
   idle_count<=0;aux_idle_count<=0;saved_valid<=0;raw_active<=0;switch_count<=0;timeout_count<=0;failed_count<=0;
   aux_context<=0;matrix_valid<=0;plane_valid<=0;requested_modes<=0;committed_modes<=0;
   mode_reject<=0;last_publish_from_key<=0;factor_bank<=0;aux_open<=0;
   aux_flags_written<=0;aux_scan_valid<=0;aux_bad<=0;aux_index<=0;aux_nonzero<=0;aux_candidate_context<=0;
  end else begin
   basis_pending<=0;
   if(basis_pending&&saved_valid&&basis_word!=base_q)basis_changed<=1;
   if(state==RECEIVE&&owner==OWNER_UART)begin
    if(input_accept||input_commit)idle_count<=0;
    else if(idle_count>=UART_IDLE_CYCLES-1)begin
     state<=SEND_ABORT;transaction_open<=0;expected_index<=0;idle_count<=0;timeout_count<=increment_saturated(timeout_count);
    end else idle_count<=idle_count+1'b1;
   end else idle_count<=0;
   if(aux_open)begin
    if(aux_accept)aux_idle_count<=0;
    else if(aux_timeout_now)begin
     aux_open<=0;aux_bad<=1;mode_reject<=1;aux_idle_count<=0;timeout_count<=increment_saturated(timeout_count);
     requested_modes<=0;mode_dirty<=0;
    end else aux_idle_count<=aux_idle_count+1'b1;
   end else aux_idle_count<=0;
   if(input_accept)begin
    if(input_addr==6'h3f)begin
     expected_index<=0;tag_good<=0;mode_good<=0;basis_changed<=0;
     transaction_open<=input_data==32'h434f4d50;work_bank<=!saved_bank;
     transaction_modes<=requested_modes;transaction_factor_bank<=factor_bank;
     // A full optical configuration establishes an ordinary baseline. Source
     // observation must be explicitly requested afterwards with EC=2.
     transaction_observation<=0;requested_observation<=0;route_pending<=0;
     if(owner!=OWNER_BOOT)owner<=input_data==32'h434f4d50 ? OWNER_UART:OWNER_NONE;
     if(input_data!=32'h434f4d50)state<=SEND_ABORT;else mode_dirty<=0;
    end else if(input_addr==6'h3e&&owner!=OWNER_BOOT)begin
     if(owner==OWNER_NONE&&pose_allowed)begin
      owner<=OWNER_UART;transaction_open<=1;expected_index<=18;tag_good<=1;mode_good<=1;candidate_mode<=1;basis_changed<=0;
      work_bank<=!saved_bank;copy_index<=0;copy_limit<=18;transaction_raw<=0;
      transaction_modes<=requested_modes;transaction_factor_bank<=factor_bank;mode_dirty<=0;state<=COPY_READ;
      transaction_observation<=observation_active;
     end else begin transaction_open<=0;expected_index<=0;state<=SEND_ABORT;failed_count<=increment_saturated(failed_count);mode_reject<=1;end
    end else if(accepted_parameter)begin
     expected_index<=expected_index+1'b1;
     if(expected_index<14)begin basis_pending<=1;basis_word<=input_data;end
     if(expected_index==16)begin candidate_mode<=input_data;mode_good<=input_data==0||input_data==1;end
     if(expected_index==17)tag_good<=input_data==32'h46495348;
    end else begin transaction_open<=0;expected_index<=0;state<=SEND_ABORT;failed_count<=increment_saturated(failed_count);end
   end
   if(input_commit&&input_commit_ready)begin transaction_open<=0;transaction_raw<=candidate_mode==0;state<=PREPARE;end
   case(state)
    COPY_READ:state<=COPY_WAIT;
    COPY_WAIT:state<=COPY_WRITE;
    COPY_WRITE:begin
     if(copy_index==copy_limit-1)begin
      if(owner==OWNER_KEY)begin candidate_mode<=transaction_raw?0:1;state<=PREPARE;end else state<=RECEIVE;
     end else begin copy_index<=copy_index+1'b1;state<=COPY_READ;end
    end
    PREPARE:begin
     if(transaction_raw)begin transaction_modes<=0;state<=OUT_BEGIN;end
     else if(transaction_modes==0)state<=OUT_BEGIN;
     else if(!matrix_valid||(transaction_modes[1]&&!plane_valid)||basis_changed)begin
      state<=RECEIVE;owner<=OWNER_NONE;mode_reject<=1;failed_count<=increment_saturated(failed_count);
     end else state<=COMPOSE_START;
    end
    COMPOSE_START:state<=COMPOSE_WAIT;
    COMPOSE_WAIT:if(compose_done)begin
     if(compose_error)begin state<=RECEIVE;owner<=OWNER_NONE;mode_reject<=1;failed_count<=increment_saturated(failed_count);end
     else state<=OUT_BEGIN;
    end
    OUT_BEGIN:if(cfg_ready)begin out_index<=0;state<=OUT_READ;end
    OUT_READ:state<=OUT_WAIT;
    OUT_WAIT:begin
     out_word<=(out_index>=18&&transaction_modes!=0&&!transaction_raw)?composed_q:base_q;
     // Bit 1 is internal map identity and is never stored in the saved lens
     // baseline. It participates in the map cache key and atomic publication.
     if(out_index==16&&!transaction_raw)out_word<={30'b0,transaction_observation,1'b1};
     state<=OUT_WORD;
    end
    OUT_WORD:if(cfg_ready)begin if(out_index==26)state<=OUT_COMMIT;else begin out_index<=out_index+1'b1;state<=OUT_READ;end end
    OUT_COMMIT:if(cfg_commit_ready)begin issued_version<=config_version;building_seen<=map_building;state<=WAIT_PUBLISH;end
    WAIT_PUBLISH:begin
     if(map_building)building_seen<=1;
     if(config_version!=issued_version)begin
      raw_active<=transaction_raw;committed_modes<=transaction_raw?2'b00:transaction_modes;
      observation_active<=transaction_observation&&!transaction_raw;
      last_publish_from_key<=owner==OWNER_KEY;mode_reject<=0;
      if(!transaction_raw)begin saved_bank<=work_bank;saved_valid<=1;end
      if(owner==OWNER_KEY)switch_count<=increment_saturated(switch_count);
      if(basis_changed)begin matrix_valid<=0;plane_valid<=0;aux_context<=0;requested_modes<=0;mode_dirty<=0;end
      owner<=OWNER_NONE;state<=RECEIVE;building_seen<=0;expected_index<=0;
     end else if(building_seen&&!map_building)begin
      failed_count<=increment_saturated(failed_count);mode_reject<=1;
      owner<=OWNER_NONE;state<=RECEIVE;building_seen<=0;expected_index<=0;
     end
    end
    SEND_ABORT:if(cfg_ready)begin owner<=OWNER_NONE;state<=RECEIVE;end
    default:begin end
   endcase
   // Queue the latest requested state through UART/build ownership. Each replay
   // starts from original H_base. No automatic retry after failed admission.
   if(state==RECEIVE&&owner==OWNER_NONE&&!aux_open&&!map_building&&saved_valid&&boot_done&&
      !uart_valid&&!uart_commit&&!aux_valid&&(key_press||route_pending||(!raw_active&&mode_dirty)))begin
    owner<=OWNER_KEY;work_bank<=!saved_bank;copy_index<=0;copy_limit<=27;
    transaction_raw<=route_pending?requested_raw:(key_press?!raw_active:raw_active);
    transaction_observation<=requested_observation;
    route_pending<=0;
    transaction_modes<=requested_modes;transaction_factor_bank<=factor_bank;
    basis_changed<=0;transaction_open<=0;mode_dirty<=0;state<=COPY_READ;
   end
   // Physical requests queue while an upload is open. Explicit auxiliary
   // commands below have the final same-edge priority, in particular EA and
   // failed uploads. Timeout/basis invalidation also reject a coincident key.
   if(scan_press&&!aux_timeout_now&&!basis_publish_now)begin
    if(saved_valid&&matrix_valid&&plane_valid)begin requested_modes[1]<=!requested_modes[1];mode_dirty<=1;mode_reject<=0;end
    else begin mode_reject<=1;failed_count<=increment_saturated(failed_count);end
   end
   if(mirror_press&&!aux_timeout_now&&!basis_publish_now)begin
    if(saved_valid&&matrix_valid)begin requested_modes[0]<=!requested_modes[0];mode_dirty<=1;mode_reject<=0;end
    else begin mode_reject<=1;failed_count<=increment_saturated(failed_count);end
   end
   if(aux_accept)begin
    case(aux_addr)
     // Wire EC: explicit complete route target. 0=corrected, 1=raw,
     // 2=corrected plus source observation. Raw+observation (3) is invalid.
     6'h39:begin
      if(saved_valid&&!aux_open&&aux_data<=2)begin
       requested_raw<=aux_data[0];requested_observation<=aux_data[1];
       route_pending<=1;mode_reject<=0;
      end else reject_aux_transaction();
     end
     6'h3a:begin aux_bad<=1;reject_aux_transaction();end
     6'h3c:begin
      if(aux_data!=0&&saved_valid&&!raw_active&&requested_modes==0&&committed_modes==0)begin
       aux_open<=1;aux_flags_written<=0;aux_index<=0;aux_nonzero<=0;aux_bad<=0;aux_candidate_context<=aux_data;
      end else reject_aux_transaction();
     end
     6'h3b:begin
      if(aux_open&&!aux_flags_written&&aux_index==0&&aux_data[31:1]==0)begin aux_flags_written<=1;aux_scan_valid<=aux_data[0];end
      else reject_aux_transaction();
     end
     6'h3d:begin
      aux_open<=0;
      if(aux_open&&aux_flags_written&&aux_index==36&&!aux_bad&&aux_candidate_context==aux_data&&
         aux_nonzero[1]&&(!aux_scan_valid||(aux_nonzero[2]&&aux_nonzero[3])))begin
       factor_bank<=!factor_bank;aux_context<=aux_candidate_context;matrix_valid<=1;plane_valid<=aux_scan_valid;mode_reject<=0;
       if(!aux_scan_valid)requested_modes[1]<=0;
       if(requested_modes!=0||committed_modes!=0)mode_dirty<=1;
      end else reject_aux_transaction();
     end
     6'h3e:begin
      if(aux_data==32'h504c494e)begin
       aux_open<=0;matrix_valid<=0;plane_valid<=0;aux_context<=0;requested_modes<=0;mode_dirty<=1;mode_reject<=0;
      end else reject_aux_transaction();
     end
     6'h3f:begin
      if(saved_valid&&mode_request_valid)begin requested_modes<=aux_data[1:0];mode_dirty<=1;mode_reject<=0;end
      else reject_aux_transaction();
     end
     default:begin
      if(aux_coefficient)begin
       aux_index<=aux_index+1'b1;
       if(aux_data!=0)begin
        if(aux_index<9)aux_nonzero[0]<=1;else if(aux_index<18)aux_nonzero[1]<=1;
        else if(aux_index<27)aux_nonzero[2]<=1;else aux_nonzero[3]<=1;
       end
       if($signed(aux_data)>65536||$signed(aux_data)< -65536)aux_bad<=1;
       if(aux_index<9&&aux_data!=((aux_index==0||aux_index==4||aux_index==8)?32'd65536:0))aux_bad<=1;
      end else reject_aux_transaction();
     end
    endcase
   end
  end
 end
endmodule

// One native 17x17 multiplier; full 66-bit dot products. The same positive
// power-of-two scale is used for all nine outputs, then floor(value+0.5).
// Iterated single-bit shifts avoid a wide variable-shift barrel network.
module complete_matrix_compose_serial(
 input wire clk,rst_n,start,output wire[3:0] h_index,c_index,input wire[31:0] h_word,c_word,
 output reg result_valid,output reg[3:0] result_index,output reg[31:0] result_word,
 output reg done,error
);
 localparam IDLE=0,FETCH=1,FETCH_WAIT=2,LIMB_ISSUE=3,LIMB_WAIT=4,LIMB_ADD=5,
  SUM_STORE=6,SUM_ABS=7,SUM_MAX=8,SUM_NEXT=9,SCALE=10,
  NORM_READ=11,NORM_WAIT=12,NORM_SHIFT=13,NORM_WRITE=14,FINISH=15,
  DET_START=16,DET_A_WAIT=17,DET_A_LOAD=18,DET_B_WAIT=19,DET_B_LOAD=20,
  DET_C_WAIT=21,DET_C_LOAD=22,DET_SECOND=23,DET_ACC=24,DET_NEXT=25,DET_CHECK=26;
 reg[4:0] state;
 reg[3:0] index,norm_index;
 reg[1:0] inner;
 reg[2:0] limb,det_term;
 reg[31:0] a_word,b_word;
 reg signed[16:0] mul_a,mul_b;
 reg signed[33:0] product;
 reg signed[65:0] accumulator,norm_word;
 reg[65:0] magnitude,maximum;
 reg signed[7:0] scale_shift,norm_shift;
 reg round_bit;
 reg det_active,det_second;
 reg[31:0] third_word;
 reg[47:0] first_product;
 reg signed[65:0] determinant;
 (* syn_ramstyle="block_ram" *) reg[65:0] sums[0:15];
 reg[65:0] sum_q;
 wire[3:0] row_base=index<3?0:index<6?3:6;
 wire[1:0] column=index<3?index:index<6?index-3:index-6;
 assign h_index=row_base+inner;
 assign c_index=(inner==0?0:inner==1?3:6)+column;
 wire signed[65:0] extended_product={{32{product[33]}},product};
 wire signed[65:0] term=det_second?
  (limb==0?extended_product:limb==1||limb==3?(extended_product<<<16):limb==5?(extended_product<<<48):(extended_product<<<32)):
  (limb==0?extended_product:limb==3?(extended_product<<<32):(extended_product<<<16));
 wire signed[65:0] next_accumulator=accumulator+term;
 wire[31:0] rounded_word=norm_word[31:0]+round_bit;
 function[3:0] det_address;input[2:0] t;input[1:0] operand;
 begin
  case(t)
   0:det_address=operand==0?0:operand==1?4:8;
   1:det_address=operand==0?1:operand==1?5:6;
   2:det_address=operand==0?2:operand==1?3:7;
   3:det_address=operand==0?2:operand==1?4:6;
   4:det_address=operand==0?1:operand==1?3:8;
   default:det_address=operand==0?0:operand==1?5:7;
  endcase
 end endfunction
 always @(posedge clk)begin
  product<=mul_a*mul_b;sum_q<=sums[norm_index];
  if(state==SUM_STORE)sums[index]<=accumulator;
  else if(state==NORM_WRITE)sums[norm_index]<={34'b0,rounded_word};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;index<=0;norm_index<=0;inner<=0;limb<=0;a_word<=0;b_word<=0;
   mul_a<=0;mul_b<=0;accumulator<=0;norm_word<=0;magnitude<=0;maximum<=0;
   scale_shift<=16;norm_shift<=0;round_bit<=0;result_valid<=0;result_index<=0;result_word<=0;done<=0;error<=0;
   det_active<=0;det_second<=0;det_term<=0;third_word<=0;first_product<=0;determinant<=0;
  end else begin
   result_valid<=0;done<=0;
   case(state)
    IDLE:if(start)begin
     index<=0;inner<=0;limb<=0;accumulator<=0;maximum<=0;scale_shift<=16;error<=0;
     det_active<=0;det_second<=0;state<=FETCH;
    end
    FETCH:state<=FETCH_WAIT;
    FETCH_WAIT:begin a_word<=h_word;b_word<=c_word;limb<=0;state<=LIMB_ISSUE;end
    LIMB_ISSUE:begin
     if(det_second)begin
      if(limb==0||limb==3)mul_a<={1'b0,first_product[15:0]};
      else if(limb==1||limb==4)mul_a<={1'b0,first_product[31:16]};
      else mul_a<={first_product[47],first_product[47:32]};
      mul_b<=limb<3?{1'b0,third_word[15:0]}:{third_word[31],third_word[31:16]};
     end else begin
      mul_a<=limb==1||limb==3?{a_word[31],a_word[31:16]}:{1'b0,a_word[15:0]};
      mul_b<=limb==2||limb==3?{b_word[31],b_word[31:16]}:{1'b0,b_word[15:0]};
     end
     state<=LIMB_WAIT;
    end
    LIMB_WAIT:state<=LIMB_ADD;
    LIMB_ADD:begin
     accumulator<=next_accumulator;
     if(det_active&&((!det_second&&limb==3)||(det_second&&limb==5)))begin
      if(det_second)state<=DET_ACC;
      else begin first_product<=next_accumulator[47:0];state<=DET_SECOND;end
     end else if(!det_active&&limb==3)begin
      if(inner==2)state<=SUM_STORE;else begin inner<=inner+1'b1;state<=FETCH;end
     end else begin limb<=limb+1'b1;state<=LIMB_ISSUE;end
    end
    SUM_STORE:state<=SUM_ABS;
    SUM_ABS:begin magnitude<=accumulator[65]?(~$unsigned(accumulator)+66'd1):$unsigned(accumulator);state<=SUM_MAX;end
    SUM_MAX:begin if(magnitude>maximum)maximum<=magnitude;state<=SUM_NEXT;end
    SUM_NEXT:begin
     if(index==8)state<=SCALE;else begin index<=index+1'b1;inner<=0;limb<=0;accumulator<=0;state<=FETCH;end
    end
    SCALE:begin
     if(maximum==0)begin error<=1;state<=FINISH;end
     else if(maximum>=66'd8589934592)begin maximum<=maximum>>1;scale_shift<=scale_shift+1'b1;end
     else if(maximum<66'd4294967296)begin maximum<=maximum<<1;scale_shift<=scale_shift-1'b1;end
     else begin norm_index<=0;state<=NORM_READ;end
    end
    NORM_READ:state<=NORM_WAIT;
    NORM_WAIT:begin norm_word<=$signed(sum_q);norm_shift<=scale_shift;round_bit<=0;state<=NORM_SHIFT;end
    NORM_SHIFT:begin
     if(norm_shift>0)begin round_bit<=norm_word[0];norm_word<=norm_word>>>1;norm_shift<=norm_shift-1'b1;end
     else if(norm_shift<0)begin round_bit<=0;norm_word<=norm_word<<<1;norm_shift<=norm_shift+1'b1;end
     else state<=NORM_WRITE;
    end
    NORM_WRITE:begin
     result_valid<=1;result_index<=norm_index;result_word<=rounded_word;
     if(norm_index==8)state<=DET_START;else begin norm_index<=norm_index+1'b1;state<=NORM_READ;end
    end
    // Check the actual quantized H, including reflections. Three Q16 words
    // have at most 54 significant product bits after normalization, so 66 bits
    // cover all six determinant terms. Threshold matches host |det(H)|>=1e-8.
    DET_START:begin determinant<=0;det_term<=0;norm_index<=0;det_active<=1;det_second<=0;state<=DET_A_WAIT;end
    DET_A_WAIT:state<=DET_A_LOAD;
    DET_A_LOAD:begin a_word<=sum_q[31:0];norm_index<=det_address(det_term,1);state<=DET_B_WAIT;end
    DET_B_WAIT:state<=DET_B_LOAD;
    DET_B_LOAD:begin b_word<=sum_q[31:0];norm_index<=det_address(det_term,2);state<=DET_C_WAIT;end
    DET_C_WAIT:state<=DET_C_LOAD;
    DET_C_LOAD:begin third_word<=sum_q[31:0];accumulator<=0;limb<=0;det_second<=0;state<=LIMB_ISSUE;end
    DET_SECOND:begin accumulator<=0;limb<=0;det_second<=1;state<=LIMB_ISSUE;end
    DET_ACC:begin
     if(det_term<3)determinant<=determinant+accumulator;else determinant<=determinant-accumulator;
     state<=DET_NEXT;
    end
    DET_NEXT:begin
     if(det_term==5)state<=DET_CHECK;
     else begin det_term<=det_term+1'b1;norm_index<=det_address(det_term+1'b1,0);det_second<=0;state<=DET_A_WAIT;end
    end
    DET_CHECK:begin error<=determinant> -66'sd2814750&&determinant<66'sd2814750;state<=FINISH;end
    FINISH:begin done<=1;state<=IDLE;end
    default:begin error<=1;state<=FINISH;end
   endcase
  end
 end
endmodule

module complete_key_debounce #(parameter integer STABLE_CYCLES=2000000)(input wire clk,rst_n,key_n,output reg press);
 function integer bits_for;input integer value;integer n;
 begin bits_for=1;for(n=value-1;n>1;n=n>>1)bits_for=bits_for+1;end endfunction
 localparam COUNT_BITS=bits_for(STABLE_CYCLES);
 (* syn_preserve=1, syn_srlstyle="registers" *) reg key_meta,key_sync;
 reg armed,stable_high;reg[COUNT_BITS-1:0] count;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin key_meta<=0;key_sync<=0;armed<=0;stable_high<=1;count<=0;press<=0;end
 else begin
  key_meta<=key_n;key_sync<=key_meta;press<=0;
  if(!armed)begin
   if(!key_sync)count<=0;else if(count>=STABLE_CYCLES-1)begin armed<=1;stable_high<=1;count<=0;end else count<=count+1'b1;
  end else if(key_sync==stable_high)count<=0;
  else if(count>=STABLE_CYCLES-1)begin stable_high<=key_sync;count<=0;if(!key_sync)press<=1;end else count<=count+1'b1;
 end
endmodule
