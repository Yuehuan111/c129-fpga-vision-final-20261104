"""Reproducible OFFLINE C128 source-inset/50-corner/quantized-H evidence.

The only saved algorithm configuration is 27 words. Full-map arrays exist
temporarily in this offline reference check and are never sent or exported.
No serial/camera/PDS access is performed.
"""
import argparse
import binascii
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

from realtime_scan import ChessboardTracker, SourceObservation
import scan_plane as sp
from complete_geometry import configuration, load_calibration
from complete_geometry_reference import vector_fixed_map
from tile_budget import tile_budget


def source_thumbnail(image):
    """Offline model of C128 bilinear sample at (4u+1.5,4v+1.5)."""
    return np.rint((image[1::4, 1::4].astype(float)+image[1::4, 2::4]+
                    image[2::4, 1::4]+image[2::4, 2::4])/4).astype(np.uint8)


def observation_fixture(raw, source_id):
    frame = np.full((720, 1280, 3), 70, np.uint8)
    frame[:180, :320] = source_thumbnail(raw)
    prefix = struct.pack('<HIIHB', 0xC128, source_id, 1_000_000+source_id*1000, 12, 3)
    bits = np.unpackbits(np.frombuffer(prefix+struct.pack('<H', binascii.crc_hqx(prefix, 0xffff)), np.uint8), bitorder='little')
    for index, bit in enumerate(np.r_[bits, 1-bits]):
        row, col = divmod(index, 80)
        frame[180+row*4:184+row*4, col*4:col*4+4] = 235 if bit else 16
    # This artificial transport wrapper exercises the live decoder without
    # connecting a camera. The report labels the source IDs as synthetic.
    return dict(frame=frame, captured_at=0., frame_id=f'offline-fixture:{source_id}',
                generation='offline', source_kind='camera', live=True)


def check_quantized_geometry(words):
    entries, domain = vector_fixed_map(1280, 720, words[:18], words[18:])
    full = np.asarray(entries, np.uint64).reshape(720, 1280).copy()
    main = np.ones((720, 1280), bool)
    main[:192, :320] = False
    valid_main = int(np.count_nonzero((full[main] >> np.uint64(38)) != 0))
    v, u = np.mgrid[:180, :320]
    uq = (4*u+1)*256+128
    vq = (4*v+1)*256+128
    full[:180, :320] = (np.uint64(1)<<np.uint64(38)) | (uq.astype(np.uint64)<<np.uint64(19)) | vq.astype(np.uint64)
    # Metadata is generated locally; its black/white cells need no source ROI.
    full[180:192, :320] = 0
    budget = tile_budget(full, 1280, 720, 64, 16, 256, 64)
    return dict(admitted=valid_main>0 and not budget['unsupported_tiles'], main_valid_pixels=valid_main,
        main_invalid_pixels=int(main.sum())-valid_main, full_unoverlaid_domain_invalid_pixels=int(domain),
        inset_sampling='(4*u+1.5,4*v+1.5), same source frame, 2x2 bilinear average',
        footer_source_reads=False, tile_budget=budget,
        scope='Offline exact integer main map plus defined source inset; not HDL/PDS/board proof')


def run(output):
    root = Path(__file__).resolve().parent
    source = root/'assets/demo_photo.jpg'
    data = source.read_bytes()
    image = sp._cv2().imdecode(np.frombuffer(data, np.uint8), sp._cv2().IMREAD_COLOR)
    raw = sp._cv2().resize(image, (1280, 720), interpolation=sp._cv2().INTER_AREA)
    cal = load_calibration()
    ctx = sp.make_context(cal, configuration(cal)[:18])
    tracker = ChessboardTracker(ctx, clock=lambda: .02, smoothing=1.)
    first = tracker.process(SourceObservation.from_capture(observation_fixture(raw, 1)), now=.01)
    result = tracker.process(SourceObservation.from_capture(observation_fixture(raw, 2)), now=.02)
    candidate = result.pop('candidate')
    if candidate is None:
        raise ValueError('Provided still-image chain did not produce an admitted visual candidate: '+str(result))
    words = configuration(cal, matrix=candidate['H'])
    geometry = check_quantized_geometry(words)
    report = dict(schema='c128_visual_static_chain_v1', source=str(source), source_sha256=hashlib.sha256(data).hexdigest(),
        source_input_size=[1920, 1080], full_fov_assumption='Existing saved photo mode assumption; no newly measured calibration',
        resize_coordinate_relation='720p source p=(1080p p+0.5)*2/3-0.5; area raster resampling for this offline fixture',
        scope='Provided still image wrapped in synthetic in-band metadata; NO live capture, board optical test or throughput claim',
        source_ids_are_synthetic=True, hardware_accessed=False, serial_written=False, map_exported=False,
        acquisition_first=first['state'], observation=candidate['observation'], quality=candidate['quality'],
        context_hash=ctx['context_hash'], calibration_sha256=ctx['calibration_sha256'],
        target_grid=candidate['target_grid'], H=candidate['H'], configuration_words27=words,
        source_points_720=candidate['source_points_720'], geometry=geometry,
        attribution_note='The footer binds the raw inset to its capture ID/time. Future visual matrix output uses a later source frame; source and output IDs must not be equated.')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[1]/'validation_c128/visual_static_chain.json')
    args = parser.parse_args()
    report = run(args.out)
    print(json.dumps(dict(output=str(args.out), quality=report['quality'], admitted=report['geometry']['admitted'],
                         tile_budget=report['geometry']['tile_budget']), ensure_ascii=False))
