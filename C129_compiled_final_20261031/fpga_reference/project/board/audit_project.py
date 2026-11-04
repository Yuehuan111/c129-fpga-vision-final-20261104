"""Check generated project dependency closure and physical pin uniqueness."""
from pathlib import Path
import re,json
from prepare_board import block
root=Path(__file__).resolve().parent
reports=[]
for channel in (1,2):
    folder=root/f'build_cmos{channel}'
    pds=(folder/'camera_board50.pds').read_text(encoding='utf-8')
    assert '(_family Logos)' in pds and '(_device PGL50H)' in pds and '(_package FBG484)' in pds
    deps=[]
    for name in ('wgt_my_design_src','wgt_my_ips_src','wgt_import_logic_con_file'):
        deps+=re.findall(r'\(_(?:file|ip|ip_source_item)\s+"([^"]+)"',block(pds,'(_widget '+name))
    for p in deps: assert (folder/p).is_file(),p
    top=(folder/'source/rtl/camera_board50_top.v').read_text(encoding='utf-8')
    assert f'parameter CAMERA_CHANNEL={channel}' in top
    ports=set()
    for declaration in re.findall(r'^\s*(?:input|output|inout)\s+([^\n]+)',top,re.M):
        declaration=re.sub(r'//.*','',declaration).strip().rstrip(',')
        match=re.search(r'(\w+)\s*$',declaration)
        if match: ports.add(match[1])
    fdc=(folder/'camera_board50_top.fdc').read_text(encoding='utf-8')
    assert 'LVCMOS12' not in fdc and '{PAP_IO_VCCIO} {1.2}' not in fdc
    pins={};assigned=set()
    for signal,pin in re.findall(r'define_attribute \{p:([^}]+)\} \{PAP_IO_LOC\} \{([^}]+)\}',fdc):
        assert pin not in pins or pins[pin]==signal,(pin,signal,pins.get(pin))
        assert signal.split('[')[0] in ports,signal
        pins[pin]=signal;assigned.add(signal.split('[')[0])
    assert ports-assigned==set(),('Unassigned ports',ports-assigned)
    assert '_pclk_16bit' not in fdc and 'u_pll.clkout3' not in fdc
    assert pins['R8']=='uart_rx' and pins['A5']=='result_led' and pins['F7']=='error_led' and pins['F8']=='capture_led'
    reports.append({'channel':channel,'dependencies':len(deps),'physical_pins':len(pins),'all_top_ports_have_locations':True})
(root/'output').mkdir(exist_ok=True)
(root/'output/project_audit.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
print('PROJECT AUDIT PASS',reports)
