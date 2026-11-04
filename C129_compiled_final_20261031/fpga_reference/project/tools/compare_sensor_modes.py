"""Read-only OV5640 register-window comparison; no device access."""
import json,re,sys
from pathlib import Path

def extract(path):
    text=path.read_bytes().decode('latin1')
    # Only actual numbered case writes, excluding commented historical tables.
    writes=re.findall(r"(?im)^\s*\d+\s*:\s*reg_data\s*<=\s*24'h([0-9a-f]{6})",text)
    registers={int(word[:4],16):int(word[4:],16) for word in writes}
    return registers

def describe(path,regs):
    def pair(a):return regs[a]*256+regs[a+1] if a in regs and a+1 in regs else None
    return {'source':str(path),'analog_x_start':pair(0x3800),'analog_y_start':pair(0x3802),
            'analog_x_end':pair(0x3804),'analog_y_end':pair(0x3806),
            'output_width':pair(0x3808),'output_height':pair(0x380a),
            'offset_x':pair(0x3810),'offset_y':pair(0x3812),
            'hts':pair(0x380c),'vts':pair(0x380e),
            'registers':{f'0x{x:04x}':f'0x{regs[x]:02x}' for x in
                         (0x3814,0x3815,0x3820,0x3821,0x4300,0x501f,0x5000,0x5001) if x in regs}}

def main():
    root=Path(__file__).resolve().parents[1]
    sys.path.insert(0,str(root/'board'))
    from sensor_1080_profile import PROFILE
    report=[]
    for p in (Path(r'D:\FPGA_Learn\12_contest2_50k\board\build_cmos1\source\rtl\reg_config.v'),
              Path(r'D:\FPGA_Learn\13_board_ready\board\build_cmos1\source\rtl\reg_config.v'),
              Path(r'D:\FPGA_Learn\09_board50_baseline\10_HDMI_DDR3_OV5640_test\source\rtl\reg_config.v')):
        report.append(describe(p,extract(p)))
    report.append(describe(root/'board/sensor_1080_profile.py',dict(PROFILE)))
    result={'comparisons':report,'conclusion':'Current wide-native1080 profile retains the supplied analog window, removes binning, doubles ISP offsets and enables3/4 sensor downscale; modeled physical field matches old720. Exact scaler convention/FOV still needs physical validation.',
            'calibration_mode_status':'User supplied the original720 capture project and explicitly authorized retaining C97 mapping/interpolation/parameters under the assumption of the requested native wide-view1080 input. No optical match or30fps board claim.',
            'required_checks':['analog window','output dimensions','offset','binning/increments','flip/mirror','sensor and HDMI/capture scaling']}
    target=root/'output/fisheye/sensor_geometry_compare.json'
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
