"""Read-only per-mode capture history with explicit asynchronous provenance."""
from copy import deepcopy
import json
from pathlib import Path
import time

from demo_controls import board_status


class ModeResults:
    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.results = {}

    def remember(self, mode, adaptive, packet, data):
        if not packet or not packet.get('live') or packet.get('source_kind') != 'camera':
            return False
        detail_active = bool((data.get('detail') or {}).get('active'))
        if mode == 'raw' and detail_active:
            return False
        key = (mode, bool(adaptive), detail_active)
        if mode == 'normal' and adaptive:
            raise ValueError('Ordinary correction cannot be enhanced')
        board = board_status(data)
        shown = (data.get('scan') or {}).get('displayed') or {}
        if not shown or shown.get('version') != board.get('config_version'):
            return False
        image = packet['frame'].copy()
        image.setflags(write=False)
        self.results[key] = {'image': image, 'mode': mode, 'adaptive': bool(adaptive),
                             'capture_id': packet['frame_id'], 'captured_at': packet['captured_at'],
                             'source': 'HDMI capture card', 'config_version': board.get('config_version'),
                             'output_evidence': deepcopy(shown), 'pose_mode': data.get('pose_mode'),
                             'crop_profile': deepcopy(data.get('crop_profile')),
                             'detail_enhancement': deepcopy(data.get('detail')),
                             'last_detail_command_evidence': deepcopy(data.get('detail_control')),
                             'view_context': (data.get('scan') or {}).get('profile_context_hash'),
                             'association': 'Asynchronous capture and serial evidence; no same-frame identity claim',
                             'capture_to_output_attribution_verified': False,
                             'saved_at': self.clock()}
        return True

    def snapshot(self, mode, adaptive, *, current_mode=None, live=False, detail=False):
        value = self.results.get((mode, bool(adaptive), bool(detail)))
        if value is None:
            return None
        result = deepcopy(value)
        result['stale'] = (not live or mode != current_mode or self.clock() - result['captured_at'] > 1.5)
        return result

    def export(self, directory):
        """Export separate images and metadata; enhanced raw never replaces I0."""
        import cv2
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        records = []
        for (mode, adaptive, detail), saved in self.results.items():
            name = f'{mode}_adaptive_{int(adaptive)}' + ('_detail_1' if detail else '')
            ok, encoded = cv2.imencode('.png', saved['image'])
            if not ok:
                raise ValueError('Unable to encode mode snapshot')
            (directory / (name + '.png')).write_bytes(encoded.tobytes())
            metadata = {key: deepcopy(value) for key, value in saved.items() if key != 'image'}
            metadata.update(image=name + '.png', stale=True, exported_at=self.clock())
            (directory / (name + '.json')).write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
            records.append(metadata)
        (directory / 'index.json').write_text(json.dumps({'schema': 'c128_mode_history_v1', 'results': records,
            'same_frame_comparison': False, 'meaning': 'Historical asynchronous capture snapshots; not five live outputs'},
            ensure_ascii=False, indent=2), encoding='utf-8')
        return len(records)
