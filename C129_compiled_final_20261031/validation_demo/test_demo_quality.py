"""SOFTWARE TEST FIXTURES ONLY: generated files are never physical-camera evidence."""
import sys,unittest,tempfile,json,hashlib,math,time
from pathlib import Path
from unittest.mock import patch
root=Path(sys.argv[1]);sys.path.insert(0,str(root/'host'))
from demo_metrics import DemoMetrics
BUILD=0x0720C129 if 'C129' in root.name else 0x0720C128

class QualityTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name);self.m=DemoMetrics(root)
  self.source=self.path/'engineering_fixture_image.png'
  from PIL import Image
  Image.new('RGB',(64,64),(100,120,140)).save(self.source)
  self.sha=hashlib.sha256(self.source.read_bytes()).hexdigest()
 def tearDown(self):self.temp.cleanup()
 def write(self,r):
  p=self.path/'engineering_fixture_report.json';p.write_text(json.dumps(r),encoding='utf-8');return p
 def line(self):return dict(measurement='manual_point_line_straightness',point_count=3,units='pixels',fit='orthogonal_total_least_squares',rms_residual=math.sqrt(2/3),max_absolute_residual=1,residuals=[-1,0,1],image=str(self.source),source='manual-points-on-image')
 def edge(self):return dict(measurement='approximate_slanted_edge_sfr',iso_certified=False,source='real-chart',image=str(self.source),image_sha256=self.sha,frequency_units='cycles_per_pixel_along_edge_normal',roi=[0,0,32,32],transfer='srgb',mtf50_cycles_per_pixel=.18,mtf50_status='measured',rows_used=32,linear_contrast=.6)
 def test_empty_values_are_pending_and_pds_visible(self):
  r=dict(self.m.observe(None,None,now=1)['rows'])
  self.assertIn('待测',r['原图直线度（离线）']);self.assertIn('待测',r['矫正后近似 MTF50（离线）']);self.assertIn('ns',r['PDS 慢角 setup（离线）'])
 def test_existing_manual_schema(self):
  self.m.load_quality_report(self.write(self.line()),role='original')
  values=dict(self.m.observe(None,None,now=1)['rows']);self.assertIn('0.816',values['原图直线度（离线）'])
  self.assertIn('来源关联未核验',values['原图直线来源'])
 def test_existing_checker_schema(self):
  line=self.line();r=dict(measurement='checkerboard_geometry',source=str(self.source),source_sha256=self.sha,row_lines=[line],column_lines=[line],line_rms_max_px=line['rms_residual'])
  self.m.load_quality_report(self.write(r));self.assertEqual(self.m._quality['corrected']['line']['values']['line_count'],2)
 def test_existing_edge_schema_and_no_crossing(self):
  r=self.edge();self.m.load_quality_report(self.write(r))
  self.assertIn('0.1800',dict(self.m.observe(None,None,now=1)['rows'])['矫正后近似 MTF50（离线）'])
  r.update(mtf50_status='not_crossed_below_nyquist',mtf50_cycles_per_pixel=None);self.m.load_quality_report(self.write(r))
  self.assertIn('未测得',dict(self.m.observe(None,None,now=1)['rows'])['矫正后近似 MTF50（离线）'])
 def test_reject_synthetic_unknown_nan_bad_hash(self):
  reports=[{},dict(self.edge(),source='synthetic-test'),dict(self.edge(),synthetic=True),dict(self.edge(),mtf50_cycles_per_pixel=float('nan')),dict(self.edge(),image_sha256='0'*64),dict(self.edge(),mtf50_cycles_per_pixel=.7),dict(self.line(),rms_residual=8)]
  for report in reports:
   with self.subTest(report=report):
    with self.assertRaises(ValueError):self.m.load_quality_report(self.write(report))
 def test_build_mismatch_label(self):
  r=self.m.observe(dict(connected={'fpga':True},fpga={'status_age_s':0},latest_pose_status={'build_id':0x1234}),None,now=1)
  self.assertIn('不匹配',r['note'])
 def test_export_preserves_report_bytes(self):
  reportpath=self.write(self.edge());raw=reportpath.read_bytes();self.m.load_quality_report(reportpath)
  log=self.path/'session.jsonl';log.write_text('')
  d=dict(session_id='fixture',connected={'fpga':True},fpga={'status_age_s':0},log_path=str(log),framing_counters={})
  with patch('demo_metrics.time.monotonic',return_value=100):self.m.start_recording(d)
  with patch('demo_metrics.time.monotonic',return_value=101):out=self.m.export(self.path,d)
  self.assertEqual((out/'quality_reports/corrected_edge.json').read_bytes(),raw)
  self.assertEqual(json.loads((out/'summary.json').read_text(encoding='utf-8'))['offline_quality_reports']['corrected']['edge']['report_sha256'],hashlib.sha256(raw).hexdigest())

unittest.main(argv=[sys.argv[0]],verbosity=2)
