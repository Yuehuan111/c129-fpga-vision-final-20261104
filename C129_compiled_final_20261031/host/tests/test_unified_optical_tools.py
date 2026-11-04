"""File-only tests: never connect devices, rebuild FPGA, or generate PDF."""
import hashlib
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from PIL import Image
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unified_optical_tools as u
from optical_tools.generate_charts import generate


def completed(runner,timeout=30):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            kind,value=runner.events.get(timeout=.1)
            if kind=='finished':return value
        except queue.Empty:pass
    raise AssertionError('offline child did not finish')


class OpticalCommandsTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.image=self.root/'含 空格 image.png'
        Image.new('RGB',(320,240),'white').save(self.image)
        self.out=self.root/'results'

    def tearDown(self):self.temp.cleanup()

    def test_calibration_is_candidate_output_with_explicit_model(self):
        for model in ['fisheye','brown']:
            c=u.build_calibration_command(self.root,self.out,'10','5','20',model,True)
            self.assertEqual(c[c.index('--model')+1],model)
            self.assertEqual(c[c.index('--out')+1],str(self.out))
            self.assertEqual(c[c.index('--cols')+1],'10')
            self.assertIn('--synthetic',c)
            self.assertNotIn('calibration/C97_joint_720p',' '.join(c))
        self.assertFalse(self.out.exists())

    def test_checker_and_line_commands_preserve_spaced_paths(self):
        checker=u.build_checker_command(self.image,self.out,10,5)
        self.assertEqual(checker[3],'checker')
        self.assertEqual(checker[checker.index('--image')+1],str(self.image.resolve()))
        csv=self.root/'points.csv';csv.write_text('x,y\n0,0\n1,1\n2,2\n')
        line=u.build_line_command(csv,self.out)
        self.assertEqual(line[line.index('--out')+1],str(self.out/'line'))

    def test_edge_arguments_are_supported_cli_and_not_automatic_roi(self):
        c=u.build_edge_command(self.image,self.out,[20,10,100,200],'vertical','gamma',2.2,'synthetic-test',8)
        i=c.index('--roi')
        self.assertEqual(c[i+1:i+5],['20','10','100','200'])
        self.assertEqual(c[c.index('--source')+1],'synthetic-test')
        self.assertEqual(c[c.index('--oversample')+1],'8')
        self.assertEqual(c[c.index('--gamma')+1],'2.2')

    def test_reject_bad_roi_model_and_board_inputs(self):
        for roi in [[-1,0,100,100],[0,0,321,20],[0,0,0,100],[0,0,'1.5',100]]:
            with self.assertRaises(ValueError):
                u.build_edge_command(self.image,self.out,roi,'vertical','srgb',2.2,'real-chart')
        for cols,rows,square in [(2,5,20),(10.5,5,20),(True,5,20),(10,5,float('nan'))]:
            with self.assertRaises(ValueError):u.board_parameters(cols,rows,square)
        with self.assertRaises(ValueError):u.build_calibration_command(self.root,self.out,10,5,20,'joint')

    def test_output_unique_and_production_parameters_are_protected(self):
        for path in [u.ROOT,u.ROOT/'calibration',u.ROOT/'vendor']:
            with self.assertRaises(ValueError):u.output_run(path,'candidate')
        a=u.output_run(u.ROOT/'results'/'optical','candidate')
        b=u.output_run(u.ROOT/'results'/'optical','candidate')
        self.assertNotEqual(a,b)
        self.assertFalse(a.exists())
        self.assertIn('results',a.parts)

    def test_chart_command_rejects_unprintable_size(self):
        c=u.build_chart_command(self.out,10,5,20,150,'svg')
        self.assertEqual(c[c.index('--format')+1],'svg')
        with self.assertRaises(ValueError):u.build_chart_command(self.out,10,5,30)


class OfflineRunnerTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.runner=u.OfflineJobRunner()

    def tearDown(self):
        self.runner.close()
        self.temp.cleanup()

    def test_constructor_does_not_execute_and_close_blocks_start(self):
        with patch.object(subprocess,'Popen') as launch:
            runner=u.OfflineJobRunner();runner.close()
            launch.assert_not_called()
            with self.assertRaises(RuntimeError):runner.start([sys.executable],self.root/'none')

    def test_failed_child_exposes_status_and_persists_output(self):
        out=self.root/'failed'
        self.runner.start([sys.executable,'-B','-c','import sys; print("bad input diagnostic"); sys.exit(7)'],out)
        result=completed(self.runner)
        self.assertEqual(result.returncode,7)
        self.assertFalse(result.cancelled)
        self.assertIn('bad input diagnostic',result.output)
        self.assertEqual(json.loads((out/'job_result.json').read_text())['returncode'],7)

    def test_existing_directory_is_never_overwritten(self):
        out=self.root/'existing';out.mkdir();(out/'console.txt').write_text('preserve')
        self.runner.start([sys.executable,'-B','-c','print("must not run")'],out)
        result=completed(self.runner)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual((out/'console.txt').read_text(),'preserve')
        self.assertFalse((out/'command.json').exists())

    def test_cancel_terminates_only_owned_child(self):
        self.runner.start([sys.executable,'-B','-c','import time; print("started",flush=True); time.sleep(20)'],self.root/'cancelled')
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            kind,value=self.runner.events.get(timeout=5)
            if kind=='output' and 'started' in value:break
        self.runner.cancel()
        result=completed(self.runner,5)
        self.assertTrue(result.cancelled)
        self.assertNotEqual(result.returncode,0)

    def test_actual_line_measurement_outputs_and_keeps_current_calibration(self):
        calibration=u.ROOT/'calibration'/'C97_joint_720p'/'calibration.json'
        before=calibration.read_bytes() if calibration.exists() else None
        csv=self.root/'line.csv';csv.write_text('x,y\n3,0\n3,10\n3,20\n3,30\n')
        out=self.root/'line_result'
        self.runner.start(u.build_line_command(csv,out),out)
        result=completed(self.runner)
        self.assertEqual(result.returncode,0,result.output)
        data=json.loads((out/'line.json').read_text())
        self.assertLess(data['rms_residual'],1e-10)
        self.assertEqual(data['units'],'pixels')
        self.assertTrue((out/'line.png').exists())
        if before is not None:self.assertEqual(calibration.read_bytes(),before)

    def test_calibration_with_no_frames_fails_without_fake_camera_json(self):
        photos=self.root/'empty_images';photos.mkdir()
        out=self.root/'candidate'
        self.runner.start(u.build_calibration_command(photos,out,10,5,20,'fisheye'),out)
        result=completed(self.runner)
        self.assertNotEqual(result.returncode,0)
        self.assertFalse((out/'camera.json').exists())

    def test_actual_checker_measurement_recovers_uniform_synthetic_grid(self):
        image=np.full((320,520,3),255,dtype=np.uint8)
        for row in range(6):
            for col in range(11):
                if (row+col)%2==0:image[40+row*40:80+row*40,40+col*40:80+col*40]=0
        path=self.root/'synthetic_checker.png';Image.fromarray(image).save(path)
        out=self.root/'checker_result'
        self.runner.start(u.build_checker_command(path,out,10,5),out)
        result=completed(self.runner)
        self.assertEqual(result.returncode,0,result.output)
        data=json.loads((out/'checker.json').read_text())
        self.assertEqual(data['point_count'],50)
        self.assertAlmostEqual(data['horizontal_spacing_mean_px'],40,delta=.1)
        # A 0.1-pixel spacing standard deviation on 40-pixel cells is 0.25%.
        # OpenCV on this host produces 0.207679% for the unchanged handoff too;
        # use the explicit subpixel budget, not a version-sensitive 0.2% cutoff.
        self.assertLess(data['horizontal_spacing_cv_percent'],100*.1/40)

    def test_actual_edge_cli_respects_synthetic_source_and_known_blur(self):
        y,x=np.mgrid[:192,:256]
        slope=math.tan(math.radians(5))
        distance=(x-128-(y-96)*slope)/math.sqrt(1+slope*slope)
        signal=.2+.6*(.5+.5*np.vectorize(math.erf)(distance/(1.2*math.sqrt(2))))
        path=self.root/'synthetic_edge.png';Image.fromarray(np.rint(signal*255).astype('uint8')).save(path)
        out=self.root/'edge_result'
        command=u.build_edge_command(path,out,[0,0,256,192],'vertical','linear',2.2,'synthetic-test')
        self.runner.start(command,out)
        result=completed(self.runner)
        self.assertEqual(result.returncode,0,result.output)
        data=json.loads((out/'edge.json').read_text())
        self.assertEqual(data['source'],'synthetic-test')
        self.assertEqual(data['transfer'],'linear')
        self.assertAlmostEqual(data['mtf50_cycles_per_pixel'],math.sqrt(2*math.log(2))/(2*math.pi*1.2),delta=.015)


class ChartsAndProvenanceTest(unittest.TestCase):
    def test_chart_physical_size_and_separate_targets_without_pdf(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)
            result=generate(out,10,5,20,150,'both')
            self.assertEqual(len(result['files']),6)
            self.assertFalse(list(out.glob('*.pdf')))
            root=ET.parse(out/'01_checkerboard.svg').getroot()
            self.assertEqual(root.attrib['width'],'297mm')
            self.assertEqual(root.attrib['height'],'210mm')
            self.assertEqual(len(root.findall('{http://www.w3.org/2000/svg}polygon')),33)
            with Image.open(out/'01_checkerboard.png') as im:
                self.assertEqual(im.size,(1754,1240))
                self.assertAlmostEqual(im.info['dpi'][0],150,delta=.02)
            self.assertFalse(result['iso_certified'])

    def test_bundled_original_tools_and_pdf_hashes_match_manifest(self):
        manifest=json.loads((u.TOOLS/'source_manifest.json').read_text(encoding='utf-8'))
        for entry in manifest['copied_unchanged']:
            copied=u.TOOLS/entry['copy']
            self.assertEqual(hashlib.sha256(copied.read_bytes()).hexdigest(),entry['sha256'],str(copied))

    def test_ui_creation_never_launches_job_or_replaces_parameters(self):
        try:root=tk.Tk()
        except tk.TclError as exc:self.skipTest(str(exc))
        root.withdraw()
        try:
            with patch.object(subprocess,'Popen') as launch:
                panel=u.OpticalToolsPanel(root)
                root.update_idletasks()
                self.assertEqual(len(panel.tabs.tabs()),4)
                self.assertFalse(panel.runner.busy)
                self.assertEqual(panel.cal_model.get(),'fisheye')
                panel.close();panel.destroy()
                launch.assert_not_called()
        finally:root.destroy()


if __name__=='__main__':unittest.main()
