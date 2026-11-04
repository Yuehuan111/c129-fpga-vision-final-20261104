"""Build a native1080 RGB565 candidate preserving the user's 720p field.

The original vendor reg_config remains intact; append a standby/mode/stream
override sequence to its generated copy. Public integration API:
    text = apply_profile(text)
Do not infer sensor ACK or actual PCLK/frame rate from generated Verilog.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import re
from pathlib import Path

MODE_ID = "ov5640_dvp_rgb565_wide_1920x1080_14_candidate_v2"
PROFILE = [
    (0x3008, 0x42),  # software standby while changing the full mode
    (0x3017, 0xFF), (0x3018, 0xFF), (0x300E, 0x58),
    # U27 XVCLK24MHz; VCO672MHz, post-bit-divider168MHz, SCLK84MHz.
    # FORCE manual DVP /2: RGB565 auto division must not silently select
    # 168MHz byte PCLK. Expected84MHz requires physical readback/measurement.
    (0x3034, 0x18), (0x3035, 0x11), (0x3036, 0x54),
    (0x3037, 0x13), (0x3108, 0x01),
    # Preserve old vflip bits[2:1]=11, clear ONLY binning bit0 (47 ->46).
    (0x3820, 0x46), (0x3821, 0x00),
    (0x3814, 0x11), (0x3815, 0x11),  # full sampling on both axes
    # EXACT old analog window2624x1456. Double the old binned offsets16/4
    # to32/8 at full sampling: effective physical2560x1440 field unchanged.
    # Sensor ISP downscales2560x1440 ->1920x1080, not720p enlargement.
    (0x3800, 0x00), (0x3801, 0x00), (0x3802, 0x00), (0x3803, 0xFA),
    (0x3804, 0x0A), (0x3805, 0x3F), (0x3806, 0x06), (0x3807, 0xA9),
    (0x3808, 0x07), (0x3809, 0x80), (0x380A, 0x04), (0x380B, 0x38),
    # 4000 byte clocks/physical line give3840 output bytes +160 margin.
    # VTS1500 covers1456 native rows. Conservative initial mode84M/6M=14fps.
    (0x380C, 0x0F), (0x380D, 0xA0), (0x380E, 0x05), (0x380F, 0xDC),
    (0x3810, 0x00), (0x3811, 0x20), (0x3812, 0x00), (0x3813, 0x08),
    (0x3618, 0x04), (0x3612, 0x2B), (0x3708, 0x64),
    (0x3709, 0x12), (0x370C, 0x00), (0x4001, 0x02),
    (0x4004, 0x06), (0x4005, 0x1A),
    (0x3002, 0x1C), (0x3006, 0xC3),  # JPEG reset/clock off
    (0x4300, 0x60), (0x501F, 0x01),  # BGR565 high byte first; FPGA swapsR/B
    (0x5000, 0xA7), (0x5001, 0xA3),  # old colour ISP + enable scaling bit5
    (0x460B, 0x37), (0x460C, 0x22), (0x3824, 0x02),
    (0x4740, 0x20), (0x4745, 0x00),
    # Fixed frame-period intent: no night extension; expose within VTS-4.
    (0x3A00, 0x78), (0x3A02, 0x05), (0x3A03, 0xD8),
    (0x3A14, 0x05), (0x3A15, 0xD8),
    (0x3A08, 0x00), (0x3A09, 0xD2), (0x3A0A, 0x00), (0x3A0B, 0xAF),
    (0x3A0D, 0x08), (0x3A0E, 0x07), (0x3503, 0x00),
    (0x4201, 0x00), (0x4202, 0x00), (0x4901, 0x00), (0x4902, 0x00),
    (0x503D, 0x00),  # normal sensor image, not colour-bar test
    (0x3008, 0x02),
]

def profile_metadata() -> dict:
    return {
        "mode_id": MODE_ID, "hardware_verified": False,
        "status": "wide_native1080_rgb565_candidate_offline_only",
        "interface": "8-bit DVP sensor D[9:2]", "format": "BGR565 high byte first; FPGA swaps R/B",
        "width": 1920, "height": 1080, "target_new_frames_per_second": 30,
        "configured_sensor_rate_hz_estimate": 14,
        "xvclk_hz_assumed_from_module_schematic": 24000000,
        "candidate_sclk_hz": 84000000, "candidate_pclk_hz": 84000000,
        "pclk_division": {"method": "manual", "0x460C": "0x22", "0x3824": "0x02"},
        "hts": 4000, "vts": 1500, "computed_sensor_rate_hz": 84000000 / (4000 * 1500),
        "analog_crop": {"x": 0, "y": 250, "width": 2624, "height": 1456},
        "isp_offset": {"x": 32, "y": 8},
        "sampling_increments": ["0x11", "0x11"], "binning": False,
        "orientation": {"0x3820": "0x46", "0x3821": "0x00", "vflip_bits_preserved": True},
        "pre_scale_dimensions": [2560, 1440], "sensor_scale_ratio": [3, 4],
        "fpga_upscale_from_720p": False,
        "reference_capture": {
            "archive": "D:/下载/10_HDMI_DDR3_OV5640_test (1).zip",
            "archive_sha256": "182448ee1c6b8fd615fea2660d1f4c7f0d23934e08310e196e0d000505c1c839",
            "reg_config_sha256": "3fb595d4000aaaaedf9261551613c7c35d79dd5db69e174ef901ea8c9f1a9469",
            "original_output": [1280, 720], "original_binning": True,
            "original_isp_offset": [16, 4],
            "effective_physical_field_model": {"x": 32, "y": 258, "width": 2560, "height": 1440}
        },
        "calibration_basis": "User authorizes retaining current C97 mapping/interpolation/parameters and treating them as the desired wide-view1080 input; optical match is not yet board-verified.",
        "warnings": [
            "Native full-sampling sensor downscale candidate; actual1920x1080 framing/FOV/colour/PCLK/14fps estimate require board validation.",
            "30 new corrected frames/s remains a goal and is NOT achieved by a14fps camera mode.",
            "No FPGA720p enlargement. Sensor full-sampling2624x1456 window, margins32/8 and ISP3/4 downscale preserve the modeled old physical field.",
            "Original vertical flip bits preserved; exact scaler pixel-center convention requires visual verification and may need later recalibration.",
            "Previous measured84MHz RAW mode is historical clock evidence, not a measurement of this RGB565/manual-divider mode.",
            "Vendor SCCB done does not certify every ACK/readback. Camera and HDMI external timing require physical validation."
        ],
        "register_writes": [{"address": f"0x{a:04x}", "value": f"0x{v:02x}"} for a, v in PROFILE],
        "sources": [
            "User capture archive reg_config.v; identicalSHA256 to local vendor source.",
            "OmniVision OV5640 DataSheetv2.01 sections4.1/4.2/5.9/5.12/7.7/7.18 and v2.03 table8-5.",
            "OmniVision OV5640 AF Imaging Module Application GuideDVP sysclk formula pp56-57.",
            "Linux upstream ov5640_set_dvp_pclk uses3034 low-nibble8 for DVP; full-sampling changes clear binning.",
            "https://device.report/m/8b7ccce47be09c6f3f605ac0d928684a5aa6f0c733eb35742e7bc40c367e6662"
        ]
    }


def apply_profile(source: str) -> str:
    if MODE_ID in source:
        raise ValueError("Profile already applied; always start from vendor source")
    match = re.search(r"if\s*\(\s*reg_index\s*<\s*(\d+)\s*\)", source)
    if not match:
        raise ValueError("Cannot find vendor reg_index limit")
    first = int(match.group(1))
    total = first + len(PROFILE)
    if total > 512:
        raise ValueError("Register table exceeds existing 9-bit index")
    source = source[:match.start(1)] + str(total) + source[match.end(1):]
    anchor = re.search(r"(?m)^\s*default\s*:\s*reg_data", source)
    if not anchor:
        raise ValueError("Cannot find vendor reg_data default")
    addition = f"\n// {MODE_ID}; HARDWARE_VERIFIED=0\n"
    addition += "\n".join(f"  {first+i}: reg_data <= 24'h{a:04x}{v:02x};" for i, (a,v) in enumerate(PROFILE)) + "\n"
    return source[:anchor.start()] + addition + source[anchor.start():]

def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--metadata", type=Path)
    args = p.parse_args()
    if args.source.resolve() == args.output.resolve():
        raise SystemExit("Refusing to replace source; choose a generated output path")
    raw = args.source.read_bytes()
    # Vendor source uses legacy Chinese comments; preserve arbitrary bytes.
    result = apply_profile(raw.decode("latin1"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(result.encode("latin1"))
    meta = profile_metadata()
    meta["vendor_source_sha256"] = hashlib.sha256(raw).hexdigest()
    meta["generated_source_sha256"] = hashlib.sha256(args.output.read_bytes()).hexdigest()
    target = args.metadata or args.output.with_suffix(".profile.json")
    target.write_text(json.dumps(meta, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "metadata": str(target), "hardware_verified": False}))

if __name__ == "__main__":
    main()
