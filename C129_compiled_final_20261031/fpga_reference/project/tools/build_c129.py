"""Full PDS compile with immutable C125-input guards and source-bound evidence."""
from pathlib import Path
from datetime import datetime, timezone
import argparse, ctypes, hashlib, json, os, shutil, subprocess, sys, winreg

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'board/build_cmos1'
OUT = ROOT / 'output/c129'
sys.path.insert(0, str(ROOT / 'output/enhancement_20261024'))
from build_support import ARTIFACTS, resources, timing
from clean_pnr_inputs import remove_pnr_input_options

ALLOWED = {'camera_board50_top.v','complete_key_config_arbiter.v','uart_config_rx_complete.v',
           'uart_status_latency_tx.v','map_build_controller_complete.v','fast_warp_engine.v',
           'video_frame_system_complete.v','complete_image_enhance.v',
           'latency_display_observer.v','latency_frame_tracker.v','video_display_burst_latency.v'}

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def now(): return datetime.now(timezone.utc).astimezone().isoformat()
def save(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def guard():
    base=json.loads((OUT/'baseline/build_state.json').read_text(encoding='utf-8'))['sources']
    expected=base['inputs']
    inputs={p.relative_to(ROOT).as_posix():sha(p) for p in sorted((BUILD/'source').rglob('*')) if p.is_file()}
    additions=inputs.keys()-expected.keys()
    assert not expected.keys()-inputs.keys(),'Required production input removed'
    assert additions<= {'board/build_cmos1/source/rtl/detail_enhance_rgb565.v'},additions
    changed=[k for k in expected if expected[k]!=inputs[k]]
    assert all(k.removeprefix('board/build_cmos1/source/rtl/') in ALLOWED for k in changed),changed
    for p in (ROOT/'rtl').glob('*.v'):
        assert sha(p)==sha(BUILD/'source/rtl'/p.name),'Source mirror: '+p.name
    assert sha(ROOT/'board/camera_board50_top.v')==sha(BUILD/'source/rtl/camera_board50_top.v')
    assert sha(BUILD/'camera_board50_top.fdc')==sha(OUT/'baseline/camera_board50_top.fdc'),'Original clocks/IO constraints changed'
    top=(ROOT/'board/camera_board50_top.v').read_text(encoding='utf-8').lower()
    build_id='0x0720C129' if "32'h0720c129" in top else '0x0720C128'
    assert "32'h0720c128" in top or "32'h0720c129" in top,'Build identity missing'
    for name in ('bilinear_rgb565_pipeline.v','complete_fisheye_mapper.v','fisheye_math.v',
                 'reg_config.v','video_config_boot.v','video_config_boot_complete.v'):
        rel='board/build_cmos1/source/rtl/'+name
        assert inputs[rel]==expected[rel],'Frozen camera/lens/interpolation changed: '+name
    return dict(inputs=inputs,changed=changed,added=sorted(additions),build_id=build_id,
                fdc_sha256=sha(BUILD/'camera_board50_top.fdc'),builder_sha256=sha(Path(__file__)))


def audit(state):
    assert state['status']=='COMPILED' and state['sources']==guard()
    for rel,item in state['artifacts'].items():
        assert sha(BUILD/rel)==item['sha256']
    after_r=resources(BUILD/'place_route/camera_board50_top.prr')
    after_t=timing(BUILD/'report_timing/rtr.db')
    baseline=json.loads((OUT/'baseline/result.json').read_text(encoding='utf-8'))['after']
    timing_ok=all(v['rows'] and all(r['slack_ns']>=0 and r['failing_endpoints']==0 and r['total_negative_slack_ns']==0 for r in v['rows'])
                  for c in after_t['corners'].values() for v in c.values())
    skew_ok=all(after_t['cdc'][c]['groups']==baseline['timing']['cdc'][c]['groups']==16 and
                {(a['assertion'],a['limit_ns']) for a in after_t['cdc'][c]['assertions']}==
                {(a['assertion'],a['limit_ns']) for a in baseline['timing']['cdc'][c]['assertions']} and
                all(a['slack_ns']>=0 and a['actual_skew_ns']<=a['limit_ns'] for a in after_t['cdc'][c]['assertions'])
                for c in ('slow','fast'))
    coverage_ok=all(after_t['coverage'].get(k)==0 for k in ('no_clock','unexpandable_clocks','loops','latch_loops','latchs'))
    fits=all(after_r[k]['used']<=after_r[k]['available'] for k in ('CLM','CLMA','CLMS','APM','DRM','LUT','FF'))
    result=dict(status='PASS' if timing_ok and skew_ok and coverage_ok and fits else 'FAIL',build_id=state['sources']['build_id'],
                physical_board_tested=False, created_at=now(), sources=state['sources'], artifacts=state['artifacts'],
                all_declared_timing_checks_passed=timing_ok,cdc_checks_passed=skew_ok,clock_coverage_passed=coverage_ok,
                resources_fit_device=fits,after=dict(resources=after_r,timing=after_t),
                resource_delta={k:after_r[k]['used']-baseline['resources'][k]['used'] for k in ('CLM','LUT','FF','APM','DRM')},
                build_attempt=state['attempt'], original_clocks_constraints_unchanged=True,added_io_constraints=[],
                external_io_signoff='Same existing external-camera IO coverage limits as C125; not physical IO signoff.')
    save(OUT/'result.json',result)
    save(ROOT/state['attempt']/'result.json',result)
    print(json.dumps({k:result[k] for k in ('status','resource_delta','all_declared_timing_checks_passed','resources_fit_device')}),flush=True)
    if result['status']!='PASS': raise SystemExit(2)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,default=12)
    ap.add_argument('--mode',choices=['normal','performance'],default='performance')
    ap.add_argument('--hold-target-ps',type=int,default=100,
                    help='PDS hold-repair target in ps; does not relax timing constraints')
    ap.add_argument('--route-hold-target-ps',type=int,default=200,
                    help='PDS in-route clock-tree hold target in ps (0..200)')
    ap.add_argument('--check-only',action='store_true')
    ap.add_argument('--audit-only',action='store_true')
    args=ap.parse_args()
    assert 0 <= args.hold_target_ps <= 1000, 'Hold-repair target must be 0..1000 ps'
    assert 0 <= args.route_hold_target_ps <= 200, 'In-route clock-tree target must be 0..200 ps'
    if args.audit_only:
        return audit(json.loads((OUT/'build_state.json').read_text(encoding='utf-8')))
    source=guard()
    if args.check_only:
        print(json.dumps(dict(status='READY',changed=source['changed']),indent=2));return
    pds=Path(os.environ.get('PDS_SHELL','D:/PDS_2022.2-SP6.4/bin/pds_shell.exe'))
    assert pds.is_file(),str(pds)
    attempt=OUT/'attempts'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    attempt.mkdir(parents=True)
    shutil.copytree(BUILD/'source',attempt/'source')
    shutil.copy2(BUILD/'camera_board50_top.fdc',attempt/'camera_board50_top.fdc')
    project=BUILD/'camera_board50.pds'
    original=project.read_bytes();cleaned,cleanup=remove_pnr_input_options(original)
    (attempt/'before.pds').write_bytes(original);project.write_bytes(cleaned)
    save(attempt/'project_cleanup.json',cleanup)
    # Keep the PCF in place: it is a required project input. Snapshot old reports
    # separately, then require every new result to be newer than this launch.
    for rel in ARTIFACTS:
        if (BUILD/rel).is_file():
            dest=attempt/'previous'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(BUILD/rel,dest)
    script='''proc checked {args} {
 puts "C128_STEP_BEGIN $args"; flush stdout
 if {[catch {uplevel 1 $args} result]} {puts stderr "C128_FAILED $args : $result";exit 1}
 puts "C128_STEP_DONE $args";flush stdout
}
'''
    script+=f'checked open_project {{{project.as_posix()}}}\n'
    script+='checked compile -force_to_run -top_module camera_board50_top\n'
    script+='checked synthesize -force_to_run -ads -selected_syn_tool_opt 2\nchecked dev_map -force_to_run\n'
    # Verified against this installed PDS version's `pnr -help`: this option
    # requires Boolean_value (TRUE/FALSE), unlike -fix_hold_violation.
    script+=f'checked pnr -force_to_run -gplace_seed {args.seed} -mode {args.mode} -optimize_multi_corner_timing -fix_hold_violation -fix_hold_violation_threshold {args.hold_target_ps} -fix_hold_violation_in_route TRUE -fix_hold_violation_threshold_for_clk_tree {args.route_hold_target_ps}\n'
    script+='checked report_timing -force_to_run -report_max_skew\nchecked gen_netlist -force_to_run\nchecked gen_bit_stream -force_to_run -master_configuration_clock_frequency {40M}\nputs "C129_FULL_FLOW_COMPLETED"\nexit\n'
    (attempt/'flow.tcl').write_text(script,encoding='utf-8')
    env=os.environ.copy()
    if not env.get('PANGO_LICENSE_FILE'):
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER,'Environment') as key:
                env['PANGO_LICENSE_FILE']=winreg.QueryValueEx(key,'PANGO_LICENSE_FILE')[0]
        except OSError: pass
    state=dict(status='STARTING',started_at=now(),sources=source,attempt=attempt.relative_to(ROOT).as_posix(),seed=args.seed,mode=args.mode,hold_target_ps=args.hold_target_ps,route_hold_target_ps=args.route_hold_target_ps)
    def record(): save(OUT/'build_state.json',state);save(attempt/'build_state.json',state)
    record();ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    try:
        with (attempt/'pds.log').open('wb') as log:
            proc=subprocess.Popen([str(pds)],cwd=BUILD,env=env,stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            state.update(status='RUNNING',pid=proc.pid);record();print('C128_PDS_STARTED',proc.pid,attempt,flush=True)
            proc.communicate(script.encode('utf-8'))
        assert proc.returncode==0 and 'C129_FULL_FLOW_COMPLETED' in (attempt/'pds.log').read_text(errors='replace')
        assert guard()==source,'Source changed during PDS run'
        artifacts={}
        for rel in ARTIFACTS:
            p=BUILD/rel
            assert p.is_file() and p.stat().st_mtime>=datetime.fromisoformat(state['started_at']).timestamp(),'Missing/stale artifact: '+rel
            artifacts[rel]=dict(sha256=sha(p),bytes=p.stat().st_size)
        state.update(status='COMPILED',finished_at=now(),artifacts=artifacts);record();audit(state)
    except Exception as exc:
        state.update(status='FAILED',error=str(exc),finished_at=now());record();raise
    finally: ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)

if __name__=='__main__':main()
