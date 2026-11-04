"""Observe the named HDMI USB capture device; never open the built-in webcam."""
import argparse
import hashlib
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
from pygrabber.dshow_graph import FilterGraph

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=45)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
names = FilterGraph().get_input_devices()
if names.count('USB Video') != 1:
    raise SystemExit(f'Expected exactly one USB Video device; found {names}')
args.output.mkdir(parents=True, exist_ok=True)
cap = cv2.VideoCapture(names.index('USB Video'), cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit('USB Video could not be opened')
mjpg_request_accepted=cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
started = time.monotonic()
count = failures = transitions = 0
previous_hash = None
changes = []
last_frame = None
try:
    while time.monotonic() - started < args.seconds:
        ok, frame = cap.read()
        if not ok:
            failures += 1
            time.sleep(0.02)
            continue
        if frame.shape[:2] != (1080, 1920):
            raise RuntimeError(f'Expected unscaled 1920x1080 capture, received {frame.shape[1]}x{frame.shape[0]}')
        count += 1
        digest = hashlib.sha256(frame.tobytes()).hexdigest()
        if digest != previous_hash:
            transitions += previous_hash is not None
            elapsed = time.monotonic() - started
            changes.append({'seconds': round(elapsed, 3), 'sha256': digest})
            if len(changes) <= 6:
                cv2.imwrite(str(args.output / f'change_{len(changes):03}.png'), frame)
            previous_hash = digest
        last_frame = frame
finally:
    cap.release()
if last_frame is not None:
    cv2.imwrite(str(args.output / 'last.png'), last_frame)
result = {
    'recorded_at': datetime.now().astimezone().isoformat(),
    'device': 'USB Video', 'dshow_index': names.index('USB Video'),
    'requested_capture_dimensions': [1920, 1080],
    'requested_usb_codec':'MJPG','codec_request_accepted':mjpg_request_accepted,
    'elapsed_seconds': time.monotonic() - started,
    'decoded_frames': count, 'read_failures': failures,
    'dimensions': list(last_frame.shape[1::-1]) if last_frame is not None else None,
    'image_transitions': transitions, 'changes': changes,
    'note': 'Decoded frame cadence is not FPGA processing FPS. Image changes alone do not prove correction quality.'
}
(args.output / 'observation.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps({k: v for k, v in result.items() if k != 'changes'}, indent=2))
