"""Derive a separately-labelled720 configuration; never alter the C97 originals."""
import hashlib
import json
from fractions import Fraction
from pathlib import Path
from fisheye_config import normalize, configuration
from project_profile import ROOT, WIDTH, HEIGHT, SENSOR_MODE, DEFAULT_CALIBRATION

SOURCE_ROOT = ROOT / 'calibration/C97_20261018_215216'
# Use the original floating-point result, not a previously Q16.16-rounded K.
DEFAULT_SOURCE = SOURCE_ROOT / 'original_run_20261018_215216_091698/calibration.json'
C_REFERENCE_HEX = SOURCE_ROOT / 'parameters/config_fisheye18.hex'


def verify_source_bundle(folder=SOURCE_ROOT):
    folder = Path(folder).resolve()
    manifest = json.loads((folder / 'SOURCE_MANIFEST.json').read_text(encoding='utf-8'))
    for entry in manifest['files']:
        path = (folder / entry['path']).resolve()
        if not path.is_relative_to(folder):
            raise ValueError('C source manifest contains an escaping path')
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError('Preserved C source/parameter hash mismatch: ' + entry['path'])
    return manifest


def reference_words():
    values = [int(line, 16) for line in C_REFERENCE_HEX.read_text(encoding='ascii').split()]
    if len(values) != 18:
        raise ValueError('The preserved C reference must contain all 18 parameter words')
    return [value if value < 0x80000000 else value - 0x100000000 for value in values]


def derive(source):
    source = Path(source).resolve()
    original = json.loads(source.read_text(encoding='utf-8-sig'))
    saved = normalize(original)
    if (saved['width'], saved['height']) != (1920, 1080):
        raise ValueError('Expected the preserved1920x1080 C97 source')
    ratio = Fraction(2, 3)
    def matrix(k):
        # Pixel centers: p720=(p1080+0.5)*(2/3)-0.5, preserving centered K_out.
        result = [[float(v) for v in row] for row in k]
        result[0][0] *= float(ratio); result[1][1] *= float(ratio)
        result[0][2] = (result[0][2] + .5) * float(ratio) - .5
        result[1][2] = (result[1][2] + .5) * float(ratio) - .5
        return result
    result = {
        'model': saved['model'], 'width': WIDTH, 'height': HEIGHT,
        'image_size': [WIDTH, HEIGHT], 'output_image_size': [WIDTH, HEIGHT],
        'camera_matrix': matrix(saved['camera_matrix']),
        'output_camera_matrix': matrix(saved['output_camera_matrix']),
        'dist_coeff_order': ['fish_k1','fish_k2','fish_k3','fish_k4'],
        'dist_coeffs': saved['distortion'], 'synthetic': False,
        'roi_cropped': False, 'rectification_rotation': [[1,0,0],[0,1,0],[0,0,1]],
        'sensor_mode': SENSOR_MODE, 'output_horizontal_fov_deg': 122.0,
        'calibration_kind': 'resolution_derived_from_real_C97',
        'optical_match_verified': False,
        'calibration_source_run': '20261018_215216_091698',
        'source_training_rms_1080p_px': original.get('rms_px'),
        'orientation': {
            'calibration': 'C supplied EXIF-aware/oriented training coordinates',
            'intended_fpga_input': 'C declares photos restored to FPGA input orientation',
            'additional_rotation_degrees': 0,
            'roll_degrees': 0,
            'board_orientation_verified': False,
        },
        'derivation': {
            'source': source.relative_to(ROOT).as_posix(),
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'source_image_size': [1920,1080], 'scale_numerator': 2, 'scale_denominator': 3,
            'coordinate_rule': 'p720=(p1080+0.5)*2/3-0.5',
            'assumption': 'Same field, no extra crop; original1080 capture resizing/scaler pixel-center convention requires board verification.',
            'new_calibration_measurement': False,
            'source_is_refitted_calibration': True,
            'source_run': '20261018_215216_091698',
            'quantization': 'Scale floating K and K_out first, then round once to Q16.16; D is not scaled.',
        },
    }
    old_words=configuration(saved); new_words=configuration(result)
    if old_words[8:12] != new_words[8:12]:
        raise ValueError('Fish coefficients must remain bit-identical')
    return result


def main():
    verify_source_bundle()
    result=derive(DEFAULT_SOURCE)
    if configuration(result) != reference_words():
        raise ValueError('Derived defaults differ from preserved C 18-word handoff; no output written')
    DEFAULT_CALIBRATION.parent.mkdir(parents=True,exist_ok=True)
    DEFAULT_CALIBRATION.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(DEFAULT_CALIBRATION)
    print('1280x720 derived parameters; no fresh optical measurement claimed.')


if __name__=='__main__': main()
