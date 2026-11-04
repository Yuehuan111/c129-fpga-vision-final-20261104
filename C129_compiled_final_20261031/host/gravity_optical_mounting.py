"""Installation rotation using fixed-board optical poses and IMU gravity.

g_I = R_IC R_CB g_B; g_B is one unknown fixed unit vector. Heading is
deliberately excluded because recorded camera/IMU yaw changes disagree.
This does NOT repair IMU yaw and is not a full-quaternion accuracy signoff.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from optical_mounting_fit import load_pose
from mounting_calibration import rotation_distance_deg
from imu_pose import validate_mounting_rotation


def solve(poses, convention='imu_to_world'):
    gravity = np.array([(p['Q'].T if convention=='imu_to_world' else p['Q']) @ [0.,0.,1.] for p in poses])
    cameras = np.array([p['R'] for p in poses])
    def residual(v):
        mounting = Rotation.from_rotvec(v[:3]).as_matrix()
        prediction = np.einsum('ij,njk,k->ni', mounting, cameras, v[3:])
        return np.r_[(prediction-gravity).ravel(), 10*(np.linalg.norm(v[3:])-1)]
    best = None
    for seed in ([0,0,0],[np.pi,0,0],[0,np.pi,0],[0,0,np.pi]):
        matrix = Rotation.from_rotvec(seed).as_matrix()
        gb = cameras[0].T @ matrix.T @ gravity[0]
        result = least_squares(residual, np.r_[seed,gb], max_nfev=1000,
                               ftol=1e-12,xtol=1e-12,gtol=1e-12)
        if best is None or np.linalg.norm(result.fun)<np.linalg.norm(best.fun):
            best = result
    mounting = Rotation.from_rotvec(best.x[:3]).as_matrix()
    gb = best.x[3:]/np.linalg.norm(best.x[3:])
    prediction = np.einsum('ij,njk,k->ni',mounting,cameras,gb)
    error = np.degrees(np.arccos(np.clip(np.sum(prediction*gravity,axis=1),-1,1)))
    return dict(convention=convention,R_IC=mounting.tolist(),gravity_in_board=gb.tolist(),
        errors_deg=error.tolist(),rms_deg=float(np.sqrt(np.mean(error**2))),max_deg=float(error.max()),
        jacobian_singular_values=np.linalg.svd(best.jac,compute_uv=False).tolist())


def fit_session(session, labels):
    session = Path(session)
    poses = [load_pose(session/label) for label in labels]
    if len({p['calibration_sha256'] for p in poses})!=1:
        raise ValueError('Mixed camera calibration')
    candidates = sorted([solve(poses,c) for c in ('imu_to_world','world_to_imu')],key=lambda c:c['rms_deg'])
    best, other = candidates
    loo = []
    if len(poses)>=5:
        for index in range(len(poses)):
            smaller=solve(poses[:index]+poses[index+1:],best['convention'])
            loo.append(rotation_distance_deg(np.array(best['R_IC']),np.array(smaller['R_IC'])))
    reasons=[]
    if len(poses)<5: reasons.append('At least five complete static poses required')
    if best['rms_deg']>1 or best['max_deg']>2: reasons.append('Gravity/optical residual too large')
    if not loo or max(loo)>5: reasons.append('Installation estimate unstable under leave-one-out')
    if other['rms_deg']-best['rms_deg']<3: reasons.append('Quaternion convention ambiguous')
    report=dict(passed=not reasons,reasons=reasons,candidates=candidates,labels=labels,
        leave_one_out_mounting_change_deg=loo,
        limits=dict(min_poses=5,max_rms_deg=1,max_error_deg=2,max_loo_change_deg=5,
            min_convention_separation_deg=3,independent_validation_max_deg=2),
        method='g_I=R_IC R_CB g_B, rotation-only optical/gravity mounting',
        purpose='Installation calibration excluding inconsistent sensor heading',
        full_quaternion_rotation_validation=False,imu_yaw_repaired=False,confirmed=False,
        sources=[{k:v for k,v in p.items() if k not in ('Q','R')} for p in poses])
    output=session/('gravity_fit_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    report['output_directory']=str(output)
    (output/'quality_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    if report['passed']:
        validate_mounting_rotation(best['R_IC'])
        mounting=dict(R_IC=best['R_IC'],convention=best['convention'],confirmed=False,
            note='Optical/gravity installation calibration, native camera coordinates. Sensor heading inconsistency remains; full attitude correction not validated. Independent validation pending.')
        (output/'mounting.candidate.json').write_text(json.dumps(mounting,indent=2),encoding='utf-8')
    return report


def validate(session, report_path, labels):
    report=json.loads(Path(report_path).read_text())
    if not report['passed']: raise ValueError('Training fit did not pass')
    if set(labels)&set(report['labels']): raise ValueError('Validation poses must be independent')
    best=report['candidates'][0]
    mounting=np.array(best['R_IC']); gb=np.array(best['gravity_in_board'])
    results=[]
    for label in labels:
        p=load_pose(Path(session)/label)
        if p['calibration_sha256']!=report['sources'][0]['calibration_sha256']:
            raise ValueError('Changed lens calibration')
        q=p['Q'].T if best['convention']=='imu_to_world' else p['Q']
        actual=q@np.array([0.,0.,1.]); predicted=mounting@p['R']@gb
        error=float(np.degrees(np.arccos(np.clip(predicted@actual,-1,1))))
        results.append(dict(label=label,error_deg=error,source_report_sha256=p['source_report_sha256']))
    output=dict(passed=len(results)>=2 and max(p['error_deg'] for p in results)<=2,
        results=results,max_error_deg=2,minimum_independent_poses=2,
        training_report=str(report_path),matrix_refitted=False,
        scope='Independent optical/gravity installation validation only; IMU yaw remains unvalidated')
    path=Path(report_path).parent/('validation_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    path.write_text(json.dumps(output,indent=2),encoding='utf-8')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--session',required=True,type=Path)
    p.add_argument('--labels',nargs='+',required=True)
    p.add_argument('--validate-report',type=Path)
    args=p.parse_args()
    result=validate(args.session,args.validate_report,args.labels) if args.validate_report else fit_session(args.session,args.labels)
    print(json.dumps(result,indent=2))
