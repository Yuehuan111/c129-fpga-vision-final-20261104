"""Task-local static camera/IMU pose capture; IMU receive-only, no FPGA writes.

Preview rotation is display-only. Each request saves three native frames across
one complete 1.5 s IMU window. No assertion of exposure/IMU synchronization.
"""
from pathlib import Path
import argparse
import collections
from datetime import datetime
import hashlib
import json
import re
import threading
import time

import cv2
import numpy as np
import serial

from imu_pose import ImuDecoder, quaternion_to_matrix, matrix_to_quaternion
from mounting_calibration import analyze_window, quaternion_mean, rotation_distance_deg
from scan_plane import _joint_project, _joint_unproject, GRID
from vendor.complete_geometry import normalize_joint_calibration


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


def estimate_pose(frame, calibration):
    if frame.shape != (720, 1280, 3):
        raise ValueError('Native 1280x720 BGR frame required')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, (10, 5), flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        raise ValueError('Full 10x5 chessboard not found')
    c = np.asarray(c, np.float64).reshape(50, 2)
    # Preserve board labels for rotations below 60 deg; explicit 180 deg board
    # ambiguity remains excluded by the session's limited motion protocol.
    if c[0, 0] > c[-1, 0]:
        c = c[::-1].copy()
    if np.min(c) < 8 or np.any(c > [1271, 711]):
        raise ValueError('Chessboard too close to border')
    width = np.linalg.norm(c[9] - c[0])
    if width < 180:
        raise ValueError(f'Chessboard too small: {width:.1f} px corner span')
    context = {'calibration': calibration}
    rays, inverse_error = _joint_unproject(c, context)
    objects = np.c_[GRID, np.zeros(50)].astype(np.float64)
    ok, rv, tv = cv2.solvePnP(objects, rays, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('Planar camera pose failed')
    rv, tv = cv2.solvePnPRefineLM(objects, rays, np.eye(3), None, rv, tv)
    rotation = cv2.Rodrigues(rv)[0]
    xyz = objects @ rotation.T + tv.reshape(3)
    if np.min(xyz[:, 2]) <= 0:
        raise ValueError('Chessboard behind camera')
    predicted = _joint_project(xyz[:, :2] / xyz[:, 2:], context)
    error = np.linalg.norm(predicted-c, axis=1)
    result = dict(R_CB=rotation.tolist(), t_CB_grid_units=tv.reshape(3).tolist(),
        corners=c.tolist(), reprojection_rms_px=float(np.sqrt(np.mean(error**2))),
        reprojection_max_px=float(error.max()), corner_span_px=float(width),
        inverse_roundtrip_px=inverse_error)
    result['passed'] = result['reprojection_rms_px'] <= 1.0 and result['reprojection_max_px'] <= 2.5
    return result


def analyze_pose(directory, calibration_path):
    directory = Path(directory)
    metadata = json.loads((directory/'capture.json').read_text())
    calibration_path = Path(calibration_path)
    calibration = normalize_joint_calibration(json.loads(calibration_path.read_text()))
    records = json.loads((directory/'imu_samples.json').read_text())
    quality = analyze_window(records, profile='handheld')
    reasons = [] if quality['passed'] else ['IMU window failed: '+str(quality['reasons'])]
    if quality.get('span_s', 0) < 1.2:
        reasons.append('IMU window coverage below 80 percent')
    if metadata.get('decoder_error_delta') != [0, 0, 0]:
        reasons.append('IMU decoder integrity error')
    poses = []
    for item in metadata['frames']:
        p = directory/item['file']
        try:
            pose = estimate_pose(cv2.imread(str(p)), calibration)
            pose['file'] = item['file']
            poses.append(pose)
            if not pose['passed']:
                reasons.append(item['file']+': optical reprojection error')
        except Exception as exc:
            reasons.append(item['file']+': '+str(exc))
    result = dict(passed=False, reasons=reasons, imu=quality, optical_frames=poses,
        calibration_file=str(calibration_path),
        calibration_sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
        capture_sha256=hashlib.sha256((directory/'capture.json').read_bytes()).hexdigest(),
        image_direction='native capture; preview-only rotation excluded',
        timing='Static interval only; host timestamps are not exposure synchronization')
    if len(poses) == 3:
        matrices = [np.asarray(p['R_CB']) for p in poses]
        mean = quaternion_to_matrix(quaternion_mean([matrix_to_quaternion(r) for r in matrices]))
        spread = max(rotation_distance_deg(mean, r) for r in matrices)
        result.update(R_CB=mean.tolist(), optical_spread_deg=spread)
        if spread > 1.0:
            reasons.append('Optical pose not stable across all three frames')
        delta = max(float(np.linalg.norm(np.asarray(p['corners'])-np.asarray(poses[0]['corners']),axis=1).max()) for p in poses)
        result['corner_motion_max_px'] = delta
        if delta > 3:
            reasons.append('Chessboard/camera moving within capture window')
    else:
        reasons.append('Three valid optical observations required')
    result['passed'] = not reasons
    save_json(directory/'pose_report.json', result)
    return result


def run(session, port, device):
    session = Path(session)
    session.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    lock = threading.Lock()
    samples = collections.deque(maxlen=3000)
    decoder = ImuDecoder()
    errors = []
    stream = serial.Serial(port=None, baudrate=115200, timeout=.02, xonxoff=False, rtscts=False, dsrdtr=False)
    stream.dtr = stream.rts = False
    stream.port = port
    stream.open()
    stream.reset_input_buffer()
    raw_file = (session/'stream_raw.bin').open('xb')
    sample_file = (session/'stream_samples.jsonl').open('x', encoding='utf-8')

    def reader():
        try:
            while not stop.is_set():
                chunk = stream.read(min(4096, max(1, stream.in_waiting)))
                now = time.monotonic()
                raw_file.write(chunk)
                with lock:
                    packets = decoder.feed(chunk, received_at=now)
                    for packet in packets:
                        if packet['kind'] == 'quaternion':
                            row = dict(received_at=now, quaternion=list(packet['quaternion']))
                            samples.append(row)
                            sample_file.write(json.dumps(row)+'\n')
        except Exception as exc:
            errors.append(str(exc))

    worker = threading.Thread(target=reader, daemon=True)
    worker.start()
    cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        stop.set()
        raise RuntimeError('Capture device failed to open')
    title = 'C129 camera-IMU calibration | display rotation only'
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 1280, 720)
    active = None
    notice = 'Keep board fixed; waiting for pose capture'
    last_preview = 0
    read_failures = 0
    save_json(session/'running.json', dict(port=port, device=device, receive_only=True,
        transmitted_imu_bytes=0, fpga_access=False, started=datetime.now().isoformat(),
        preview_rotate_180=True, native_size=[1280,720]))
    try:
        while not (session/'stop.request').exists():
            ok, frame = cap.read()
            now = time.monotonic()
            if not ok:
                read_failures += 1
                if read_failures >= 30:
                    raise RuntimeError('Capture frame failed repeatedly')
                time.sleep(.1)
                continue
            read_failures = 0
            if errors:
                raise RuntimeError(errors[-1])
            if frame.shape != (720,1280,3):
                raise RuntimeError('Capture did not return native resolution')
            request = session/'request.json'
            if active is None and request.exists():
                command = json.loads(request.read_text(encoding='utf-8-sig'))
                label = command['label']
                if not re.fullmatch('[a-zA-Z0-9_-]+', label):
                    raise ValueError('Invalid label')
                directory = session/label
                directory.mkdir(exist_ok=False)
                request.rename(directory/'request.json')
                with lock:
                    counters = [decoder.checksum_errors, decoder.length_errors, decoder.invalid_values]
                active = dict(label=label, path=directory, start=now, images=[], counters=counters)
                notice = 'Recording '+label+' - hold still'
            if active:
                elapsed = now-active['start']
                if len(active['images']) < 3 and elapsed >= [.1,.75,1.4][len(active['images'])]:
                    filename = 'native_'+str(len(active['images']))+'.png'
                    if not cv2.imwrite(str(active['path']/filename), frame):
                        raise RuntimeError('Image write failed')
                    active['images'].append(dict(file=filename, received_at=now))
                if elapsed >= 1.5:
                    with lock:
                        rows = [r for r in samples if active['start'] <= r['received_at'] <= now]
                        counters = [decoder.checksum_errors, decoder.length_errors, decoder.invalid_values]
                    save_json(active['path']/'imu_samples.json', rows)
                    save_json(active['path']/'capture.json',dict(label=active['label'],
                        started_at=active['start'], ended_at=now, frames=active['images'],
                        decoder_error_delta=[a-b for a,b in zip(counters,active['counters'])],
                        errors=list(errors), port=port, source='FPGA confirmed RAW mode, DSHOW native',
                        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in active['path'].iterdir() if p.is_file()}))
                    notice = 'Saved '+active['label']+' - waiting for next instruction'
                    print(notice, flush=True)
                    active = None
            view = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.putText(view, notice, (15,28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0,255,255), 2)
            cv2.imshow(title, view)
            if now-last_preview > 1:
                cv2.imwrite(str(session/'preview_current.jpg'), view)
                last_preview = now
            if cv2.waitKey(1) == 27:
                break
    finally:
        stop.set()
        worker.join(timeout=2)
        stream.close()
        raw_file.close()
        sample_file.close()
        cap.release()
        cv2.destroyAllWindows()
        save_json(session/'stopped.json',dict(errors=errors, stopped=datetime.now().isoformat()))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['run','analyze'])
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--port', default='COM21')
    parser.add_argument('--device', type=int, default=1)
    parser.add_argument('--calibration', type=Path, default=Path(__file__).parent/'calibration/C97_joint_720p/calibration.json')
    args = parser.parse_args()
    if args.command == 'run':
        run(args.session,args.port,args.device)
    else:
        report = analyze_pose(args.session,args.calibration)
        print(json.dumps({k:v for k,v in report.items() if k not in ['optical_frames']}, indent=2))
