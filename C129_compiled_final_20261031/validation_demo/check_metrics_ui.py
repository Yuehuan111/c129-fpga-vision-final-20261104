"""Offline Tk layout/recording checks; never opens a camera or serial port."""
import json
from pathlib import Path
import sys
import tempfile
import tkinter as tk
from unittest.mock import patch

project = Path(sys.argv[1]).resolve()
preferences = project / 'host/settings/demo_preferences.json'
original_preferences = preferences.read_bytes()
sys.path.insert(0, str(project / 'host'))
from demo_app import DemoControlPanel
from demo_theme import confirmed_mode

root = tk.Tk()
root.withdraw()
errors = []
root.report_callback_exception = lambda *args: errors.append(str(args))
app = DemoControlPanel(root, offline=True)
checks = []
try:
    assert not app.metrics_visible
    assert app.metrics_panel.winfo_manager() == ''
    checks.append('metrics hidden by default; original full-width video restored')
    from copy import deepcopy
    fresh = {'connected': {'fpga': True}, 'fpga': {'status_age_s': .1},
             'latest_pose_status': {'config_version': 7, 'raw_active': False, 'observation_active': False},
             'scan': {'displayed': {'version': 7, 'raw': False, 'observation': False}}}
    assert confirmed_mode(fresh, 'normal') == 'normal'
    assert confirmed_mode(fresh, 'normal', waiting=True) is None
    for alteration in ('disconnected', 'stale', 'wrong_version', 'route_mismatch', 'frozen', 'geometry_recovery'):
        state = deepcopy(fresh)
        if alteration == 'disconnected': state['connected']['fpga'] = False
        elif alteration == 'stale': state['fpga']['status_age_s'] = 3
        elif alteration == 'wrong_version': state['scan']['displayed']['version'] = 6
        elif alteration == 'route_mismatch': state['latest_pose_status']['raw_active'] = True
        else: state[alteration] = True
        assert confirmed_mode(state, 'normal') is None, alteration
    fresh['continuous'] = True
    assert confirmed_mode(fresh, 'imu') == 'imu'
    fresh['continuous'] = False
    assert confirmed_mode(fresh, 'imu') is None
    checks.append('mode emphasis requires fresh matching output; waiting/fault/stale/static IMU not highlighted')
    for width, height in ((1000, 700), (1360, 860)):
        root.geometry(f'{width}x{height}')
        root.deiconify()
        root.update()
        app.update()
        root.update_idletasks()
        hidden_width = app.video_panel.winfo_width()
        app.toggle_metrics_panel()
        root.update_idletasks()
        assert app.metrics_panel.winfo_ismapped()
        assert app.video_panel.winfo_width() < hidden_width
        right = app.metrics_panel.winfo_rootx() + app.metrics_panel.winfo_width()
        assert right <= root.winfo_rootx() + root.winfo_width(), (right, root.winfo_width())
        assert app.metrics_canvas.winfo_width() > 200
        assert str(app.metrics_start_button['state']) == 'disabled'
        assert str(app.metrics_export_button['state']) == 'disabled'
        assert len(app.metrics_widgets) >= 7
        for widget in app.metrics_toggle_button.master.winfo_children():
            assert widget.winfo_x() >= 0
            assert widget.winfo_x() + widget.winfo_width() <= widget.master.winfo_width()
        app.metrics_canvas.yview_moveto(1)
        root.update_idletasks()
        assert app.metrics_canvas.yview()[1] >= .99
        checks.append(dict(size=[width, height], video_hidden_width=hidden_width,
                           video_metrics_width=app.video_panel.winfo_width(),
                           metrics_rows=len(app.metrics_widgets), scroll_to_bottom=True))
        app.toggle_metrics_panel()
        root.update_idletasks()
        assert app.video_panel.winfo_width() == hidden_width
    try:
        app.start_metrics_recording()
        raise AssertionError('offline recording must be rejected')
    except ValueError as error:
        assert '离线' in str(error)
    with patch('demo_app.filedialog.askdirectory', return_value=''):
        assert app.export_metrics_recording() is None
    with patch('demo_app.filedialog.askopenfilename', return_value=''):
        assert app.load_quality_metrics('corrected') is None
    checks.append('offline recording rejected; cancelling export leaves state intact')
    class FakeMetrics:
        def __init__(self): self.exports = []; self.quality = []
        def start_recording(self, data, mode=None): return 'synthetic UI test started'
        def load_quality_report(self, path, role='corrected'):
            self.quality.append((path, role))
            return 'synthetic report callback'
        def export(self, directory, data=None):
            self.exports.append(str(directory))
            return Path(directory) / 'synthetic_ui_test_no_files_written'
    fake = FakeMetrics()
    app.demo_metrics = fake
    for role in ('original', 'corrected'):
        with patch('demo_app.filedialog.askopenfilename', return_value='synthetic.json'):
            assert app.load_quality_metrics(role) == 'synthetic report callback'
    assert fake.quality == [('synthetic.json', 'original'), ('synthetic.json', 'corrected')]
    checks.append('quality report menu preserves explicit original/corrected role; cancellation does nothing')
    app.offline = False  # UI callback-only test; no device/session is created.
    assert app.start_metrics_recording() == 'synthetic UI test started'
    assert app.metrics_recording
    with patch('demo_app.filedialog.askdirectory', return_value=''):
        assert app.export_metrics_recording() is None
    assert app.metrics_recording
    with tempfile.TemporaryDirectory() as directory:
        with patch('demo_app.filedialog.askdirectory', return_value=directory):
            assert '本段记录已保存' in app.export_metrics_recording()
    assert not app.metrics_recording and len(fake.exports) == 1
    app.start_metrics_recording()
    checks.append('synthetic callback test: cancelled export preserves recording; successful export ends it')
    assert not errors, errors
finally:
    app.close()
    preferences.write_bytes(original_preferences)
if 'fake' in globals():
    assert len(fake.exports) == 2 and not app.metrics_recording
    checks.append('unfinished recording auto-export requested on close')
print(json.dumps({'status': 'PASS', 'project': str(project), 'checks': checks,
                  'device_access': False, 'callback_errors': errors}, ensure_ascii=False, indent=2))
