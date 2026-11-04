// Joint calibration CANDIDATE ONLY. Do not automatically replace a tested firmware.
// Use source K + fisheye k1..k4 + tangential p1,p2 as one matched set.
// p acts after fisheye radial q and before source K. New tangential RTL is required.
// x_t=x+2*p1*x*y+p2*(3*x*x+y*y); y_t=y+p1*(x*x+3*y*y)+2*p2*x*y.
// Size 1920x1080; derived from original photos, no native-mode optical verification.
// Existing-data or numerical checks do not support automatic adoption.
localparam signed [31:0] JOINT_FX_SRC_Q16_16 = 32'sh0274E7BC;
localparam signed [31:0] JOINT_FY_SRC_Q16_16 = 32'sh027A0A3A;
localparam signed [31:0] JOINT_CX_SRC_Q16_16 = 32'sh03A38268;
localparam signed [31:0] JOINT_CY_SRC_Q16_16 = 32'sh01F716E7;
localparam signed [31:0] JOINT_FX_OUT_Q16_16 = 32'sh021422FE;
localparam signed [31:0] JOINT_FY_OUT_Q16_16 = 32'sh021422FE;
localparam signed [31:0] JOINT_CX_OUT_Q16_16 = 32'sh03BF8000;
localparam signed [31:0] JOINT_CY_OUT_Q16_16 = 32'sh021B8000;
localparam signed [31:0] JOINT_FISH_K1_Q16_16 = 32'sh00000004;
localparam signed [31:0] JOINT_FISH_K2_Q16_16 = 32'sh000000B7;
localparam signed [31:0] JOINT_FISH_K3_Q16_16 = 32'sh000000D9;
localparam signed [31:0] JOINT_FISH_K4_Q16_16 = 32'shFFFFFFB0;
localparam signed [31:0] JOINT_FISH_P1_Q16_16 = 32'shFFFFFFF3;
localparam signed [31:0] JOINT_FISH_P2_Q16_16 = 32'sh00000012;
// Final existing-data/Q16 review: keep existing formal parameters; this is a diagnostic candidate.
