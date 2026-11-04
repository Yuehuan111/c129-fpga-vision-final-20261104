"""Build a MES50HP-only PDS project from local, licensed vendor dependencies.
Never overwrites the vendor example or an unmarked/nonempty output directory.
"""
from pathlib import Path
from datetime import datetime
import argparse, hashlib, json, re, shutil
from sensor_720_profile import apply_profile, profile_metadata
from cdc_constraints import gray_cdc_fdc
from project_identity import pds_semantic_sha256

def block(text, prefix):
    start=text.index(prefix); depth=0; quote=False
    for i in range(start,len(text)):
        c=text[i]
        if c=='"': quote=not quote
        if not quote:
            if c=='(': depth+=1
            elif c==')':
                depth-=1
                if depth==0: return text[start:i+1]
    raise ValueError('Unbalanced PDS block '+prefix)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vendor',type=Path,default=Path('D:/FPGA_Learn/09_board50_baseline/10_HDMI_DDR3_OV5640_test'))
    ap.add_argument('--channel',type=int,choices=(1,2),default=1)
    ap.add_argument('--seed',type=int,default=None,help='Optional PDS placement seed; does not relax timing constraints')
    a=ap.parse_args(); root=Path(__file__).resolve().parents[1]
    src=a.vendor.resolve(); dst=root/'board'/('build_cmos'+str(a.channel))
    if src==dst or src in dst.parents: raise ValueError('Output overlaps vendor')
    if dst.exists() and any(dst.iterdir()) and not (dst/'.vision50-generated').exists():
        raise ValueError('Output is nonempty and unmarked')
    dst.mkdir(parents=True,exist_ok=True); (dst/'.vision50-generated').write_text('MES50HP generated project\n')
    manifest=[]; patches=[]
    def copy(path,rel):
        to=dst/rel; to.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(path,to)
        manifest.append({'path':rel,'source':str(path),'source_sha256':hashlib.sha256(to.read_bytes()).hexdigest()})
    def patch(rel,before,after,count=1):
        p=dst/rel; data=p.read_bytes(); old=before.encode('ascii'); new=after.encode('ascii')
        if data.count(old)!=count: raise ValueError('Unexpected patch pattern: '+rel+' / '+before)
        p.write_bytes(data.replace(old,new)); patches.append({'file':rel,'before':before,'after':after,'count':count})
    original=(src/'HDMI_DDR3_OV5640_test.pds').read_text(encoding='utf-8-sig')
    for p in sorted((root/'rtl').glob('*.v')): copy(p,'source/rtl/'+p.name)
    copy(root/'board/camera_board50_top.v','source/rtl/camera_board50_top.v')
    profile=json.loads((root/'board/hardware_profile.json').read_text(encoding='utf-8-sig'))
    if set(profile)!={'vsync_active_level'} or any(type(v)!=int or v not in (0,1) for v in profile.values()):
        raise ValueError('hardware_profile requires only vsync_active_level, integer 0 or 1')
    if profile['vsync_active_level']!=1: patch('source/rtl/camera_board50_top.v','parameter VSYNC_ACTIVE_LEVEL=1','parameter VSYNC_ACTIVE_LEVEL=0')
    if a.channel==2: patch('source/rtl/camera_board50_top.v','parameter CAMERA_CHANNEL=1','parameter CAMERA_CHANNEL=2')
    for name in ['power_on_delay.v','reg_config.v','i2c_com.v','ms72xx_ctl.v','ms7200_ctl.v','ms7210_ctl.v','iic_dri.v','sync_vg.v']:
        copy(src/'source/rtl'/name,'source/rtl/'+name)
    for file,before,after,count in [
        ('reg_config.v','always@(posedge clock_20k)','always@(posedge clock_20k or negedge camera_rstn)',1),
        # 24.74747 MHz / 2450 = 10.10101 kHz, period 99000 ns. The vendor's
        # /2500 gives 101020 ns, beyond PDS's <100000 ns clock-period limit.
        ('reg_config.v','clock_20k_cnt<1249','clock_20k_cnt<1224',1),
        ('i2c_com.v','always@(posedge clock_i2c)','always@(posedge clock_i2c or negedge camera_rstn)',2),
        # Export boot SDA's drive-low enable and read the real physical pad.
        # Top owns one explicit boot/runtime mux; no internal inout feedback.
        ('i2c_com.v','               i2c_sdat            //',
         '               i2c_sdat_in, i2c_sdat_drive_low //',1),
        ('i2c_com.v','    inout i2c_sdat;',
         '    input i2c_sdat_in;\n    output i2c_sdat_drive_low;',1),
        ('i2c_com.v','    wire i2c_sdat;',
         '    // Physical SDA input and open-drain enable are separate.',1),
        ('i2c_com.v',"assign i2c_sdat=reg_sdat?1'bz:0;",
         'assign i2c_sdat_drive_low=~reg_sdat;',1),
        ('i2c_com.v','<=i2c_sdat;','<=i2c_sdat_in;',4),
        ('reg_config.v','inout i2c_sdat,',
         'input i2c_sdat_in,\n          output i2c_sdat_drive_low,',1),
        ('reg_config.v','.i2c_sdat(i2c_sdat)',
         '.i2c_sdat_in(i2c_sdat_in),.i2c_sdat_drive_low(i2c_sdat_drive_low)',1),
        ('ms72xx_ctl.v','wire         init_over_rx;','// init_over_rx declared in ANSI port list.',1),
        ('power_on_delay.v','output initial_en','output reg initial_en',1),
        ('power_on_delay.v','reg initial_en;','// initial_en declared in ANSI port list.',1),
    ]: patch('source/rtl/'+file,before,after,count)
    sensor_path=dst/'source/rtl/reg_config.v'
    sensor_path.write_bytes(apply_profile(sensor_path.read_bytes().decode('latin1')).encode('latin1'))
    (dst/'sensor_profile.json').write_text(json.dumps(profile_metadata(),indent=2),encoding='utf-8')
    ipblocks=[]
    for name in ['pll','DDR3_50H']:
        ipblock=block(original,'(_ip "source/rtl/'+name+'/'+name+'.idf"'); ipblocks.append(ipblock)
        # Copy the complete generated IP folder, excluding old implementation results.
        folder=src/'source/rtl'/name
        for p in folder.rglob('*'):
            if not p.is_file(): continue
            rel=p.relative_to(folder)
            if any(x in ['compile','synthesize','device_map','place_route','report_timing','generate_bitstream','bak','log','logbackup'] for x in rel.parts): continue
            if p.suffix.lower() in ['.sbit','.smsk','.log','.db','.adf','.prr']: continue
            copy(p,p.relative_to(src).as_posix())
    # PDS's official PLL generator: PFD 5..320 MHz, VCO 600..1250 MHz.
    # 50/3*49=816.666667 MHz VCO; /11=74.242424 MHz pixel clock.
    # 1650x750 raster gives59.993878 Hz; camera target30fps, a -102 ppm nominal-30 approximation.
    pll_path=dst/'source/rtl/pll/pll.v'
    pll_text=pll_path.read_text(encoding='ascii')
    pll_values={'STATIC_RATIOI':3,'STATIC_RATIOF':49,'STATIC_DUTYF':49,
                'STATIC_RATIO0':11,'STATIC_DUTY0':11,'STATIC_RATIO1':82,'STATIC_DUTY1':82,
                'STATIC_RATIO2':33,'STATIC_DUTY2':33}
    for name,value in pll_values.items():
        pll_text,n=re.subn(r'(localparam integer '+name+r'\s*=\s*)\d+(;)',lambda m:m[1]+str(value)+m[2],pll_text)
        if n!=1: raise ValueError('PLL parameter pattern mismatch '+name)
    #50/3=16.667MHz PFD belongs to LOW (5..20MHz), UG021001.
    pll_text,n=re.subn(r'(localparam BANDWIDTH\s*=\s*)"OPTIMIZED"',r'\1"LOW"',pll_text)
    if n!=1: raise ValueError('PLL bandwidth template mismatch')
    # Physical A/B test: an unreset main PLL never locked; the same ratios
    # locked after a20ms reset pulse. Expose the IP reset rather than tie low.
    for before,after in [('    clkin1,','    pll_rst,\n    clkin1,'),
                         ('    input clkin1;','    input pll_rst;\n    input clkin1;'),
                         ("    assign pll_rst      = 1'b0;",'    // pll_rst is driven by the independent startup-reset circuit.')]:
        if pll_text.count(before)!=1: raise ValueError('PLL reset template mismatch: '+before)
        pll_text=pll_text.replace(before,after)
    pll_path.write_text(pll_text,encoding='ascii')
    # Keep the editable IP metadata consistent with the compiled wrapper.
    idf=dst/'source/rtl/pll/pll.idf'
    idf_text=idf.read_text(encoding='utf-8')
    for name,value in pll_values.items():
        idf_text=re.sub(r'(<name>'+name+r'_(?:basicPage|advancedPage)</name>\s*<value>)[^<]+(</value>)',
                        lambda m:m[1]+str(value)+m[2],idf_text)
    idf_text,n=re.subn(r'(<name>BANDWIDTH_(?:basicPage|advancedPage)</name>\s*<value>)OPTIMIZED(</value>)',r'\1LOW\2',idf_text)
    if n!=2: raise ValueError('PLL bandwidth IP metadata mismatch')
    idf_text,n=re.subn(r'(<name>RST_ENABLE_(?:basicPage|advancedPage)</name>\s*<value>)false(</value>)',r'\1true\2',idf_text)
    if n!=2: raise ValueError('PLL reset IP metadata mismatch')
    idf.write_text(idf_text,encoding='utf-8')
    copy(src/'hdmi_ddr_ov5640_top.fdc','camera_board50_top.fdc')
    fdc=(dst/'camera_board50_top.fdc').read_text(encoding='utf-8',errors='replace')
    # No half-rate DVP clock exists in this design. Keep the vendor DDR instance name.
    lines=[line for line in fdc.splitlines() if '_pclk_16bit' not in line]
    fdc='\n'.join(lines)+'\n'
    # Vendor FDC is stale: 67.5 MHz / clkout3 disagree with its generated PLL.
    # Ratios verified against GTP_PLL_E3: f_out=f_in*RATIOF/(RATIOI*RATIOout).
    fdc=re.sub(r'(create_generated_clock -name \{pix_clk\}[^\n]*-multiply_by )\{27\} -divide_by \{20\}',r'\g<1>{49} -divide_by {33}',fdc)
    fdc=re.sub(r'(create_generated_clock -name \{cfg_clk\}[^\n]*-multiply_by )\{1\} -divide_by \{5\}',r'\g<1>{49} -divide_by {246}',fdc)
    fdc=re.sub(r'(create_generated_clock -name \{clk_25M\}[^\n]*-multiply_by )\{1\} -divide_by \{2\}',r'\g<1>{49} -divide_by {99}',fdc)
    fdc=fdc.replace('u_pll.clkout3','u_pll.clkout2')
    # Prepared reg_config toggles clock_20k after 1225 clk_25M cycles: a full
    # cycle is 2450 cycles (10.10101 kHz with our 24.74747 MHz PLL output).
    # Keep this generated clock synchronous to its clk_25M parent, including
    # the vendor's otherwise single-clock asynchronous group declaration.
    camera_clock=('create_generated_clock -name {camera_i2c_clk} '
                  '-source [get_nets {u_pll.clkout2}] '
                  '[get_nets -hierarchical -hsc / {sensor'+str(a.channel)+'.camera_setup/clock_20k}] '
                  '-master_clock [get_clocks {clk_25M}] -divide_by {2450}')
    fdc,n=re.subn(r'(create_generated_clock -name \{clk_25M\}[^\n]*\n)',
                  lambda match:match[1]+camera_clock+'\n',fdc)
    if n!=1: raise ValueError('Missing/duplicate clk_25M definition for camera generated clock')
    fdc,n=re.subn(r'(set_clock_groups[^\n]*-group \[get_clocks )\{clk_25M\}(\])',
                  r'\g<1>{clk_25M camera_i2c_clk}\2',fdc)
    if n!=1: raise ValueError('Missing/duplicate clk_25M asynchronous group')
    fdc+='''
# MES50HP UART0_RX (hardware manual): R8, Bank 2, 3.3 V.
define_attribute {p:uart_rx} {PAP_IO_DIRECTION} {INPUT}
define_attribute {p:uart_rx} {PAP_IO_LOC} {R8}
define_attribute {p:uart_rx} {PAP_IO_VCCIO} {3.3}
define_attribute {p:uart_rx} {PAP_IO_STANDARD} {LVCMOS33}
# MES50HP UART0_TX: R9. Status telemetry, 115200 8N1.
define_attribute {p:uart_tx} {PAP_IO_DIRECTION} {OUTPUT}
define_attribute {p:uart_tx} {PAP_IO_LOC} {R9}
define_attribute {p:uart_tx} {PAP_IO_VCCIO} {3.3}
define_attribute {p:uart_tx} {PAP_IO_STANDARD} {LVCMOS33}
define_attribute {p:uart_tx} {PAP_IO_DRIVE} {4}
define_attribute {p:uart_tx} {PAP_IO_SLEW} {SLOW}
# MES50HP hardware manual section3.4.1/table3-8: KEY1, active low.
# The physical switch has a board pull-up and is synchronized/debounced in RTL.
define_attribute {p:geometry_key_n} {PAP_IO_DIRECTION} {INPUT}
define_attribute {p:geometry_key_n} {PAP_IO_LOC} {K18}
define_attribute {p:geometry_key_n} {PAP_IO_VCCIO} {3.3}
define_attribute {p:geometry_key_n} {PAP_IO_STANDARD} {LVCMOS33}
# DVP input delays require measured/module timing information.
# This file is an initial constraint set, NOT timing-signoff evidence.
'''
    fdc+=gray_cdc_fdc()
    for port,pin in [('result_led','A5'),('error_led','F7'),('capture_led','F8')]:
        for attr,value in [('PAP_IO_DIRECTION','OUTPUT'),('PAP_IO_LOC',pin),('PAP_IO_VCCIO','3.3'),('PAP_IO_STANDARD','LVCMOS33'),('PAP_IO_DRIVE','4'),('PAP_IO_SLEW','SLOW')]:
            fdc+='define_attribute {p:'+port+'} {'+attr+'} {'+value+'}\n'
    # MES50HP standard-bank supply is 3.3 V; DDR uses its separate SSTL15 bank.
    fdc=fdc.replace('{PAP_IO_VCCIO} {1.2}', '{PAP_IO_VCCIO} {3.3}').replace('{PAP_IO_STANDARD} {LVCMOS12}', '{PAP_IO_STANDARD} {LVCMOS33}')
    fdc=fdc.replace('{PAP_IO_DRIVE} {2}', '{PAP_IO_DRIVE} {4}')
    (dst/'camera_board50_top.fdc').write_text(fdc,encoding='utf-8')
    items=[]
    for p in sorted((dst/'source/rtl').glob('*.v')):
        top=' + "camera_board50_top"' if p.name=='camera_board50_top.v' else ''
        stamp=datetime.fromtimestamp(p.stat().st_mtime).strftime('%Y-%m-%dT%H:%M:%S')
        items.append('                (_file "source/rtl/'+p.name+'"'+top+'\n                    (_format verilog)\n                    (_timespec "'+stamp+'")\n                )')
    project=original.replace(block(original,'(_widget wgt_my_design_src'),'(_widget wgt_my_design_src\n            (_input\n'+'\n'.join(items)+'\n            )\n        )')
    project=project.replace(block(project,'(_widget wgt_my_ips_src'),'(_widget wgt_my_ips_src\n            (_input\n                '+'\n                '.join(ipblocks)+'\n            )\n        )')
    # PDS otherwise inserts this functional option on its first run. Record it
    # before computing project identity, rather than ignoring command options.
    if '(_option top_module' not in block(project,'(_command cmd_compile'):
        project=project.replace('(_command cmd_compile',
            '(_command cmd_compile\n            (_option top_module (_string "camera_board50_top"))',1)
    # Official PDS User Guide, pnr options: optimize hold in both corners.
    project=project.replace('(_command cmd_pnr', '(_command cmd_pnr\n            (_option optimize_multi_corner_timing (_switch ON))\n            (_option fix_hold_violation (_switch ON))\n            (_option fix_hold_violation_threshold (_integer 100))',1)
    # Confirmed by this installed PDS's report_timing -help. Native max-skew
    # assertions are otherwise omitted from the report by default.
    project=project.replace('(_command cmd_report_post_pnr_timing',
                            '(_command cmd_report_post_pnr_timing\n            (_option report_max_skew (_switch ON))',1)
    seed=a.seed if a.seed is not None else (4 if a.channel==1 else None)
    if seed is not None:
        if seed < 1: raise ValueError('Placement seed must be positive')
        project=project.replace('(_command cmd_pnr', '(_command cmd_pnr\n            (_option gplace_seed (_integer '+str(seed)+'))',1)
    project=project.replace('hdmi_ddr_ov5640_top','camera_board50_top')
    # Avoid showing old reference build outputs as if they came from this design.
    for prefix in ['(_output']:
        while prefix in project: project=project.replace(block(project,prefix),'',1)
    (dst/'camera_board50.pds').write_text(project,encoding='utf-8')
    for widget in ['wgt_my_design_src','wgt_my_ips_src','wgt_import_logic_con_file']:
        section=block(project,'(_widget '+widget)
        for rel in re.findall(r'\(_(?:file|ip|ip_source_item)\s+"([^"]+)"',section):
            if not (dst/rel).is_file(): raise ValueError('Missing dependency '+rel)
    for item in manifest: item['prepared_sha256']=hashlib.sha256((dst/item['path']).read_bytes()).hexdigest()
    (dst/'source_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    (dst/'vendor_patches.json').write_text(json.dumps(patches,indent=2),encoding='utf-8')
    # Source RTL hashes alone do not detect stale sensor/PLL/FDC generation.
    # Record generator inputs and generated project descriptors as well.
    generation_inputs=[root/'board'/name for name in (
        'prepare_board.py','sensor_720_profile.py','cdc_constraints.py','project_identity.py',
        'hardware_profile.json','boot_config.json')]
    generation_inputs.append(root/'tools/project_profile.py')
    generation_outputs=[dst/name for name in (
        'camera_board50.pds','camera_board50_top.fdc','sensor_profile.json')]
    generation_manifest={
        'channel':a.channel,'placement_seed':seed,
        'inputs':[{'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
                  for p in generation_inputs],
        'outputs':[{'path':p.relative_to(dst).as_posix(),
                    'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
                   for p in generation_outputs],
        'rtl_sources':[str(p) for p in sorted((root/'rtl').glob('*.v'))],
    }
    for item in generation_manifest['outputs']:
        if item['path']=='camera_board50.pds':
            item['semantic_sha256']=pds_semantic_sha256((dst/item['path']).read_text(encoding='utf-8-sig'))
    (dst/'generation_manifest.json').write_text(json.dumps(generation_manifest,indent=2),encoding='utf-8')
    print('PREPARED',dst/'camera_board50.pds')
    print('Logos PGL50H -6 FBG484; CMOS'+str(a.channel)+'; no synthesis/bitstream claimed')

if __name__=='__main__': main()

