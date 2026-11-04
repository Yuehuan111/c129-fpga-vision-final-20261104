"""Host/build metadata only; FPGA geometry remains synthesizable Verilog."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1280, 720
BUILD_ID = 0x0720C125
SENSOR_MODE = 'ov5640_dvp_rgb565_wide_1280x720_30_candidate_v1'
DEFAULT_CALIBRATION = ROOT / 'calibration/C97_joint_720p/calibration.json'
JOINT_MODEL = 'fisheye4_then_tangential2_joint_intrinsics'
# The UART protocol remains 27 words; these two values are FPGA build constants.
TANGENTIAL_Q16 = (-13, 18)

