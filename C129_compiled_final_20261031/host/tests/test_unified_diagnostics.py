"""No-hardware tests for explicitly gated endpoint capture."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import tkinter as tk
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unified_diagnostics as d


class CaptureArgumentsTest(unittest.TestCase):
    def test_exact_supported_cli_without_retry_or_automatic_writes(self):
        with tempfile.TemporaryDirectory() as temp:
            session=Path(temp)/'new session'
            command,endpoint=d.build_capture_command(' COM5 ',session,'x_start','handheld','1.2')
            self.assertEqual(command[3],'capture')
            self.assertEqual(command[command.index('--port')+1],'COM5')
            self.assertEqual(command[command.index('--label')+1],'x_start')
            self.assertIn('--handheld',command)
            self.assertNotIn('--retry-failed',command)
            self.assertEqual(endpoint,session/'x_start')
            self.assertFalse(session.exists())
            static,_=d.build_capture_command('COM5',session,'z_end','static',.8)
            self.assertNotIn('--handheld',static)

    def test_existing_endpoint_cannot_be_overwritten_even_if_failed(self):
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'x_start';target.mkdir()
            summary=target/'summary.json';summary.write_text('{"quality":{"passed":false}}')
            before=summary.read_bytes()
            with self.assertRaises(ValueError):d.build_capture_command('COM5',temp,'x_start','static',1.2)
            self.assertEqual(summary.read_bytes(),before)

    def test_reject_incomplete_or_invalid_capture_parameters(self):
        with tempfile.TemporaryDirectory() as temp:
            valid=['COM5',temp,'y_end','static',1.2]
            for index,value in [(0,''),(1,''),(2,'all'),(3,''),(4,.5),(4,11),(4,float('nan'))]:
                args=valid.copy();args[index]=value
                with self.subTest(args=args),self.assertRaises(ValueError):d.build_capture_command(*args)

    def test_shared_session_guard_checks_imu_and_fpga(self):
        self.assertFalse(d.session_connected(None))
        self.assertFalse(d.session_connected({'connected':{'imu':False,'fpga':False}}))
        self.assertTrue(d.session_connected({'connected':{'imu':True,'fpga':False}}))
        self.assertTrue(d.session_connected({'connected':{'imu':False,'fpga':True}}))
        self.assertTrue(d.session_connected({'imu':{'connected':True}}))

    def test_runner_construction_and_close_never_open_subprocess(self):
        with patch.object(subprocess,'Popen') as process:
            runner=d.MountingCaptureRunner();runner.close()
            process.assert_not_called()
            with self.assertRaises(RuntimeError):runner.start([sys.executable])

    def test_cancel_terminates_only_its_owned_process(self):
        runner=d.MountingCaptureRunner()
        owned=Mock();owned.poll.return_value=None
        unrelated=Mock()
        runner._process=owned
        runner.cancel()
        owned.terminate.assert_called_once()
        unrelated.terminate.assert_not_called()
        runner._process=None;runner.close()


class CapturePanelTest(unittest.TestCase):
    def setUp(self):
        try:self.root=tk.Tk()
        except tk.TclError as exc:self.skipTest(str(exc))
        self.root.withdraw();self.temp=tempfile.TemporaryDirectory();self.panels=[]

    def tearDown(self):
        for panel in self.panels:panel.close();panel.destroy()
        self.root.destroy();self.temp.cleanup()

    def panel(self,**kwargs):
        panel=d.DiagnosticsPanel(self.root,**kwargs);self.panels.append(panel)
        panel.capture_port.set('COM5');panel.capture_session.set(self.temp.name)
        panel.capture_label.set('x_start');panel.capture_profile.set('static')
        return panel

    def test_offline_mode_refuses_capture_even_direct_method_call(self):
        with patch.object(d.MountingCaptureRunner,'start') as start:
            panel=self.panel(allow_devices=False)
            self.assertFalse(panel.start_capture())
            self.assertIn('离线',panel.note.get())
            start.assert_not_called()

    def test_connected_shared_session_refuses_independent_port(self):
        with patch.object(d.MountingCaptureRunner,'start') as start:
            panel=self.panel(snapshot_provider=lambda:{'connected':{'imu':True,'fpga':False}})
            self.assertFalse(panel.start_capture())
            self.assertIn('先在连接页断开',panel.note.get())
            start.assert_not_called()

    def test_disconnected_explicit_action_starts_only_requested_endpoint(self):
        with patch.object(d.MountingCaptureRunner,'start') as start:
            panel=self.panel(snapshot_provider=lambda:None)
            start.assert_not_called()
            self.assertTrue(panel.start_capture())
            command=start.call_args.args[0]
            self.assertEqual(command[command.index('--label')+1],'x_start')
            self.assertFalse((Path(self.temp.name)/'x_start').exists())

    def test_cancel_button_targets_only_owned_runner(self):
        panel=self.panel()
        with patch.object(panel.capture_runner,'cancel') as cancel:
            # Rebind after mocking, equivalent to the explicit button action.
            panel.capture_cancel_button.configure(command=panel.capture_runner.cancel,state='normal')
            panel.capture_cancel_button.invoke();cancel.assert_called_once()

    def test_failed_capture_is_shown_without_automatic_retry(self):
        with patch.object(d.MountingCaptureRunner,'start') as start:
            panel=self.panel()
            panel.capture_endpoint=Path(self.temp.name)/'x_start'
            panel.capture_runner.events.put(('finished',{'returncode':1,'cancelled':False,'output':'No valid quaternion samples'}))
            # Process one event through the real UI reporting path.
            panel.after_cancel(panel._after);panel.poll()
            self.assertIn('失败',panel.note.get())
            self.assertIn('No valid quaternion samples',panel.report['output'])
            start.assert_not_called()


if __name__=='__main__':unittest.main()
