"""Rotation-only camera/IMU mounting from a FIXED target and static poses.

R_CB is target-to-camera, Q is IMU-to-world. For every pose pair,
A=Q_i.T Q_j, B=R_CB_i R_CB_j.T and A X = X B, X=R_IC.
No device access. All chosen captures must predate fit; validation captures
are separate and never enter the optimization. Output remains unconfirmed.
"""
import argparse
from datetime import datetime
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from imu_pose import quaternion_to_matrix, validate_mounting_rotation
from mounting_calibration import rotation_vector, rotation_distance_deg


def load_pose(path):
    path = Path(path)
    r = json.loads((path/'pose_report.json').read_text())
    if r['passed'] is not True:
        raise ValueError(str(path)+': failed capture/optical quality')
    capture = path/'capture.json'
    if hashlib.sha256(capture.read_bytes()).hexdigest() != r['capture_sha256']:
        raise ValueError('Capture metadata changed')
    metadata = json.loads(capture.read_text())
    for name, expected in metadata['hashes'].items():
        if hashlib.sha256((path/name).read_bytes()).hexdigest() != expected:
            raise ValueError('Capture source changed: '+name)
    return dict(label=path.name, Q=quaternion_to_matrix(r['imu']['mean_quaternion_wxyz']),
        R=np.asarray(r['R_CB']), calibration_sha256=r['calibration_sha256'],
        source_report_sha256=hashlib.sha256((path/'pose_report.json').read_bytes()).hexdigest())


def pairs(poses, convention):
    result = []
    for i,j in itertools.combinations(range(len(poses)),2):
        q0,q1=poses[i]['Q'],poses[j]['Q']
        if convention=='world_to_imu': q0,q1=q0.T,q1.T
        a=q0.T@q1
        b=poses[i]['R']@poses[j]['R'].T
        result.append((i,j,a,b))
    return result


def fit(poses, convention):
    all_pairs=pairs(poses,convention)
    useful=[p for p in all_pairs if 4<=np.degrees(np.linalg.norm(rotation_vector(p[3])))<=80]
    if len(useful)<3: raise ValueError('Insufficient independent rotations')
    av=np.array([rotation_vector(p[2]) for p in useful])
    bv=np.array([rotation_vector(p[3]) for p in useful])
    u,s,vt=np.linalg.svd(av.T@bv)
    x=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    def residual(v):
        m=Rotation.from_rotvec(v).as_matrix()
        return np.concatenate([rotation_vector(a.T@m@b@m.T) for _,_,a,b in all_pairs])
    solution=least_squares(residual,Rotation.from_matrix(x).as_rotvec(),max_nfev=200,
                           ftol=1e-12,xtol=1e-12,gtol=1e-12)
    x=solution.x
    mounting=Rotation.from_rotvec(x).as_matrix()
    errors=np.degrees(np.linalg.norm(residual(x).reshape(-1,3),axis=1))
    angle_errors=[abs(np.linalg.norm(rotation_vector(a))-np.linalg.norm(rotation_vector(b)))*180/np.pi for _,_,a,b in all_pairs]
    return dict(convention=convention,R_IC=mounting.tolist(),
        pair_rms_deg=float(np.sqrt(np.mean(errors**2))),pair_max_deg=float(errors.max()),
        angle_difference_rms_deg=float(np.sqrt(np.mean(np.square(angle_errors)))),
        directional_singular_values=s.tolist(),direction_ratio=float(s[1]/s[0]),
        pair_details=[dict(first=poses[i]['label'],second=poses[j]['label'],error_deg=float(e),
             imu_angle_deg=float(np.degrees(np.linalg.norm(rotation_vector(a)))),
             camera_angle_deg=float(np.degrees(np.linalg.norm(rotation_vector(b)))))
             for (i,j,a,b),e in zip(all_pairs,errors)], useful_pairs=len(useful))


def fit_session(session,labels):
    session=Path(session)
    poses=[load_pose(session/l) for l in labels]
    if len({p['calibration_sha256'] for p in poses})!=1: raise ValueError('Mixed lens calibrations')
    candidates=sorted([fit(poses,c) for c in ('imu_to_world','world_to_imu')],key=lambda r:r['pair_rms_deg'])
    best,other=candidates
    reasons=[]
    if len(poses)<5: reasons.append('At least five training poses required')
    if best['useful_pairs']<6 or best['direction_ratio']<.15: reasons.append('Insufficient independent rotation directions')
    if best['pair_rms_deg']>2 or best['pair_max_deg']>4: reasons.append('Camera/IMU relative rotation inconsistent')
    if best['angle_difference_rms_deg']>2: reasons.append('Camera/IMU rotation angles disagree')
    if other['pair_rms_deg']-best['pair_rms_deg']<1 or other['pair_rms_deg']<2*best['pair_rms_deg']:
        reasons.append('Quaternion convention not sufficiently separated')
    report=dict(passed=not reasons,reasons=reasons,candidates=candidates,confirmed=False,
        sources=[{k:v for k,v in p.items() if k not in ('Q','R')} for p in poses],
        limits=dict(min_training_poses=5,max_pair_rms_deg=2,max_pair_deg=4,
            max_angle_difference_rms_deg=2,min_direction_ratio=.15,min_convention_gap_deg=1),
        validation='Independent held-out static poses and live compensation still required',
        mounting_definition='native camera right/down/forward to IMU; preview rotation excluded')
    output=session/('fit_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir()
    if report['passed']:
        validate_mounting_rotation(best['R_IC'])
        mounting=dict(R_IC=best['R_IC'],convention=best['convention'],confirmed=False,
            note='Optical/IMU static mounting fit; independent validation pending. Native camera coordinates.')
        (output/'mounting.candidate.json').write_text(json.dumps(mounting,indent=2),encoding='utf-8')
    report['output_directory']=str(output)
    (output/'quality_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--labels',nargs='+',required=True)
    args=parser.parse_args()
    print(json.dumps(fit_session(args.session,args.labels),indent=2))
