// Synthesizable, bounded iterative arithmetic for the PDS fisheye mapper.
// No real variables, simulation math functions or combinational divide/sqrt.
// Nine registered 16-bit limb products use native-size DSP multipliers.
// Four rising-edge stages, accepting new operands on every clock; no reset/CE.
module fisheye_multiplier48(
 input wire clk,input wire signed[47:0] a,b,output reg signed[95:0] product
);
 reg[31:0] p00,p01,p10,p11;
 reg signed[31:0] p02,p12,p20,p21,p22;
 reg[15:0] low_s2,low_s3,digit1_s3;
 reg[32:0] column1_s2;
 reg signed[33:0] column2_s2,middle_s3;
 reg signed[32:0] column3_s2;
 reg signed[31:0] p22_s2;
 reg signed[48:0] high_s3;

 // Compress the three inputs before a single carry-propagating addition.
 wire[32:0] c1_a={1'b0,p01},c1_b={1'b0,p10};
 wire[32:0] c1_c={17'b0,p00[31:16]};
 wire[32:0] c1_sum=c1_a^c1_b^c1_c;
 wire[32:0] c1_carry=((c1_a&c1_b)|(c1_a&c1_c)|(c1_b&c1_c))<<1;
 wire[33:0] c2_a={{2{p02[31]}},p02},c2_b={2'b0,p11};
 wire[33:0] c2_c={{2{p20[31]}},p20};
 wire[33:0] c2_sum=c2_a^c2_b^c2_c;
 wire[33:0] c2_carry=((c2_a&c2_b)|(c2_a&c2_c)|(c2_b&c2_c))<<1;
 wire signed[48:0] high_sum=high_s3+
  $signed({{31{middle_s3[33]}},middle_s3[33:16]});

 always @(posedge clk) begin
  // Stage 1: low/middle limbs are unsigned; the top limb is signed.
  // Mixed products zero-extend the unsigned limb; their exact result fits 32 bits.
  p00<=$unsigned(a[15:0])*$unsigned(b[15:0]);
  p01<=$unsigned(a[15:0])*$unsigned(b[31:16]);
  p10<=$unsigned(a[31:16])*$unsigned(b[15:0]);
  p11<=$unsigned(a[31:16])*$unsigned(b[31:16]);
  p02<=$signed({1'b0,a[15:0]})*$signed(b[47:32]);
  p12<=$signed({1'b0,a[31:16]})*$signed(b[47:32]);
  p20<=$signed(a[47:32])*$signed({1'b0,b[15:0]});
  p21<=$signed(a[47:32])*$signed({1'b0,b[31:16]});
  p22<=$signed(a[47:32])*$signed(b[47:32]);

  // Stage 2: coefficients at weights 2^16, 2^32 and 2^48.
  low_s2<=p00[15:0];p22_s2<=p22;
  column1_s2<=c1_sum+c1_carry;
  column2_s2<=$signed(c2_sum)+$signed(c2_carry);
  column3_s2<=$signed({p12[31],p12})+$signed({p21[31],p21});

  // Stage 3: fold the low carry and the top product into separate sums.
  low_s3<=low_s2;digit1_s3<=column1_s2[15:0];
  middle_s3<=column2_s2+$signed({17'b0,column1_s2[32:16]});
  high_s3<=$signed({p22_s2[31],p22_s2,16'b0})+
   $signed({{16{column3_s2[32]}},column3_s2});

  // Stage 4: arithmetic carry from the middle coefficient completes the product.
  product<={high_sum[47:0],middle_s3[15:0],digit1_s3,low_s3};
 end
endmodule

module fisheye_unsigned_divider(
 input wire clk,rst_n,start,input wire[63:0] numerator,input wire[31:0] denominator,
 output reg busy,done,error,output reg[63:0] quotient
);
 reg[63:0] dividend;
 reg[31:0] divisor;
 reg[32:0] remainder;
 reg[5:0] iteration;
 wire[32:0] shifted={remainder[31:0],dividend[63]};
 wire subtract=shifted>={1'b0,divisor};
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin busy<=0;done<=0;error<=0;quotient<=0;dividend<=0;divisor<=0;remainder<=0;iteration<=0;end
 else begin
  done<=0;
  if(start&&!busy) begin
   quotient<=0;dividend<=numerator;divisor<=denominator;remainder<=0;iteration<=0;error<=0;
   if(denominator==0) begin busy<=0;done<=1;error<=1;end
   else busy<=1;
  end else if(busy) begin
   dividend<={dividend[62:0],1'b0};
   quotient<={quotient[62:0],subtract};
   remainder<=subtract?shifted-{1'b0,divisor}:shifted;
   if(iteration==63) begin busy<=0;done<=1;end
   else iteration<=iteration+1'b1;
  end
 end
endmodule

module fisheye_unsigned_sqrt(
 input wire clk,rst_n,start,input wire[63:0] radicand,
 output reg busy,done,output reg[31:0] root
);
 reg[63:0] operand;
 reg[35:0] remainder;
 reg[4:0] iteration;
 wire[35:0] shifted={remainder[33:0],operand[63:62]};
 wire[35:0] trial={2'b0,root,2'b01};
 wire subtract=shifted>=trial;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin busy<=0;done<=0;root<=0;operand<=0;remainder<=0;iteration<=0;end
 else begin
  done<=0;
  if(start&&!busy) begin busy<=1;root<=0;operand<=radicand;remainder<=0;iteration<=0;end
  else if(busy) begin
   operand<={operand[61:0],2'b0};
   root<={root[30:0],subtract};
   remainder<=subtract?shifted-trial:shifted;
   if(iteration==31) begin busy<=0;done<=1;end
   else iteration<=iteration+1'b1;
  end
 end
endmodule

// atan(radius), unsigned radius Q4.28 -> nonnegative radians Q2.30.
// 30 CORDIC vectoring iterations, shift and add in separate clock cycles.
module fisheye_atan(
 input wire clk,rst_n,start,input wire[31:0] radius_q28,
 output reg busy,done,output reg signed[31:0] theta_q30
);
 reg signed[35:0] vx,vy,shift_x,shift_y;
 reg signed[31:0] angle,step_angle;
 reg[4:0] iteration;
 reg phase;
 function signed[31:0] cordic_angle;
  input[4:0] index;
  begin case(index)
   0:cordic_angle=32'sd843314857;1:cordic_angle=32'sd497837829;
   2:cordic_angle=32'sd263043837;3:cordic_angle=32'sd133525159;
   4:cordic_angle=32'sd67021687;5:cordic_angle=32'sd33543516;
   6:cordic_angle=32'sd16775851;7:cordic_angle=32'sd8388437;
   8:cordic_angle=32'sd4194283;9:cordic_angle=32'sd2097149;
   10:cordic_angle=32'sd1048576;11:cordic_angle=32'sd524288;
   12:cordic_angle=32'sd262144;13:cordic_angle=32'sd131072;
   14:cordic_angle=32'sd65536;15:cordic_angle=32'sd32768;
   16:cordic_angle=32'sd16384;17:cordic_angle=32'sd8192;
   18:cordic_angle=32'sd4096;19:cordic_angle=32'sd2048;
   20:cordic_angle=32'sd1024;21:cordic_angle=32'sd512;
   22:cordic_angle=32'sd256;23:cordic_angle=32'sd128;
   24:cordic_angle=32'sd64;25:cordic_angle=32'sd32;
   26:cordic_angle=32'sd16;27:cordic_angle=32'sd8;
   28:cordic_angle=32'sd4;default:cordic_angle=32'sd2;
  endcase end
 endfunction
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin busy<=0;done<=0;theta_q30<=0;vx<=0;vy<=0;angle<=0;shift_x<=0;shift_y<=0;step_angle<=0;iteration<=0;phase<=0;end
 else begin
  done<=0;
  if(start&&!busy) begin
   busy<=1;vx<=36'sd268435456;vy<=$signed({4'b0,radius_q28});angle<=0;iteration<=0;phase<=0;
  end else if(busy) begin
   if(!phase) begin shift_x<=vx>>>iteration;shift_y<=vy>>>iteration;step_angle<=cordic_angle(iteration);phase<=1;end
   else begin
    phase<=0;
    if(vy>0) begin vx<=vx+shift_y;vy<=vy-shift_x;angle<=angle+step_angle;end
    else if(vy<0) begin vx<=vx-shift_y;vy<=vy+shift_x;angle<=angle-step_angle;end
    if(iteration==29) begin
     busy<=0;done<=1;
     theta_q30<=vy>0?angle+step_angle:vy<0?angle-step_angle:angle;
    end else iteration<=iteration+1'b1;
   end
  end
 end
endmodule
