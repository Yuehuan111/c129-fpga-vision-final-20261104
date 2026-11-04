import sys,unittest,tempfile,json,time
from pathlib import Path
from unittest.mock import patch
root=Path(sys.argv[1]);sys.path.insert(0,str(root/'host'))
from demo_metrics import DemoMetrics
BUILD=0x0720C129 if 'C129' in root.name else 0x0720C128

def stat(t=1,p=30,raw=30,ver=7):
 return dict(packet_type='status',protocol_version=5,build_id=BUILD,sample_ticks=t,processed=p,captured=p,raw_valid_frames=raw,flags=0,config_version=ver,underflow=0,overflow=0,dropped=0,camera_skipped_frames=0)
def lat(i=10,ver=7,base=1000,flags=1):
 return dict(packet_type='latency',protocol_version=6,magic=0x4c415431,source_frame_id=i,config_version=ver,flags=flags,unit_ns=1000,input_first_us=base,capture_done_us=(base+30000)&0xffffffff,warp_start_us=(base+31000)&0xffffffff,warp_done_us=(base+42000)&0xffffffff,output_first_us=(base+45000)&0xffffffff,output_last_us=(base+60000)&0xffffffff,repeated_output_frames=0,display_underflow=0,missing_capture_tags=0,missing_display_tags=0,metadata_overrun=0,uart_drops=0)
def data(s=None,l=None,at=2):
 s=s or stat()
 return dict(session_id='test-session',connected={'fpga':True},fpga={'status_age_s':.1},latest_v5_status=s,latest_v5_age_s=.1,latest_v5_received_at=at,latest_pose_status=s,latest_latency_packet=l,latest_latency_received_at=at,latest_latency_age_s=.1,log_path='unused',imu={'stale':True})
class MetricsTests(unittest.TestCase):
 def setUp(self):self.m=DemoMetrics(root)
 def test_offline(self):
  r=self.m.observe(None,None,now=1);self.assertIsNone(r['values']['new_processed_fps']);self.assertIsNone(r['values']['first_pixel_latency_ms'])
  with self.assertRaises(ValueError):self.m.start_recording(None)
 def test_rates_are_board_ticks(self):
  self.m.observe(data(stat(10,100,120)),{},'normal',now=1)
  r=self.m.observe(data(stat(12,150,180)),{},'normal',now=1.01)
  self.assertEqual(r['values']['new_processed_fps'],25);self.assertEqual(r['values']['camera_input_fps'],30)
 def test_stale_waiting_and_mode(self):
  self.m.observe(data(stat(1)),{},'normal',now=1)
  self.assertEqual(self.m.observe(data(stat(2,60,60)),{},'normal',now=2)['values']['new_processed_fps'],30)
  self.assertIsNone(self.m.observe(data(stat(3,90,90)),{},'raw',now=3)['values']['new_processed_fps'])
  d=data(stat(4,120,120));d['latest_v5_age_s']=4
  self.assertIsNone(self.m.observe(d,{},'raw',now=4)['values']['new_processed_fps'])
  self.assertIsNone(self.m.observe(data(),{},'raw',waiting=True,now=5)['values']['new_processed_fps'])
 def test_latency_duplicate_no_inflation_and_wrapped(self):
  self.m.observe(data(),{},'normal',now=1)
  d=data(l=lat(base=0xfffffff0),at=2)
  r=self.m.observe(d,{},'normal',now=2)
  self.assertEqual(r['values']['first_pixel_latency_ms'],45)
  self.m.observe(d,{},'normal',now=2.1)
  self.assertEqual(len(self.m._latency_validator.samples),1)
  d['latest_latency_age_s']=3
  self.assertIsNone(self.m.observe(d,{},'normal',now=5)['values']['first_pixel_latency_ms'])
 def test_reject_invalid_and_old_version(self):
  self.m.observe(data(),{},'normal',now=1)
  for flags in [0,3,5,9]:
   self.assertIsNone(self.m.observe(data(l=lat(i=flags,flags=flags),at=2),{},'normal',now=2)['values']['first_pixel_latency_ms'])
  self.assertIsNone(self.m.observe(data(l=lat(i=13,ver=6),at=3),{},'normal',now=3)['values']['first_pixel_latency_ms'])
 def test_counter_wrap_and_reset(self):
  self.m.observe(data(stat(0xffffffff,0xfffffff0,0xfffffff0)),{},'normal',now=1)
  self.assertEqual(self.m.observe(data(stat(0,14,14)),{},'normal',now=2)['values']['new_processed_fps'],30)
  self.assertIsNone(self.m.observe(data(stat(0xffffffff,0,0)),{},'normal',now=3)['values']['new_processed_fps'])
 def test_transition_does_not_relabel_prior_sample(self):
  self.m.observe(data(),{},'normal',now=1)
  self.m.observe(data(l=lat(),at=2),{},'normal',now=2)
  self.assertIsNone(self.m.observe(data(l=lat(),at=2),{},'raw',now=3)['values']['first_pixel_latency_ms'])
 def test_parser_errors_prevent_trust_and_record_window_excludes_prior_v5(self):
  d=data(at=9);self.m.observe(d,{},'normal',now=10)
  self.assertIsNone(self.m._previous_v5)
  self.m.observe(data(stat(2,60,60),at=11),{},'normal',now=11)
  self.assertIsNone(self.m._rates.get('observed_new_fps'))
  self.assertEqual(self.m.observe(data(stat(3,90,90),at=12),{},'normal',now=12)['values']['new_processed_fps'],30)
 def test_c129_actual_route_mismatch(self):
  if BUILD!=0x0720C129:return
  d=data();d['latest_pose_status']=dict(d['latest_pose_status'],detail_active=True,raw_active=False)
  self.m.observe(d,{},'normal',now=1)
  d.update(latest_latency_packet=lat(),latest_latency_received_at=2)
  self.assertIsNone(self.m.observe(d,{},'normal',now=2)['values']['first_pixel_latency_ms'])
 def test_percentiles_require_multiple_same_route_version_samples(self):
  self.m.observe(data(),{},'normal',now=1)
  for i in range(1,6):
   r=self.m.observe(data(l=lat(i=i),at=i+1),{},'normal',now=i+1)
   if i<5:self.assertIsNone(r['values']['first_pixel_latency_p95_ms'])
  self.assertEqual(r['values']['latency_same_version_route_samples'],5)
  self.assertEqual(r['values']['first_pixel_latency_p50_ms'],45)
  self.assertEqual(r['values']['complete_frame_latency_p95_ms'],60)
  d=data(stat(ver=8),lat(i=6,ver=8),at=7)
  self.assertIsNone(self.m.observe(d,{},'normal',now=7)['values']['first_pixel_latency_p95_ms'])
 def test_export_raw_window_and_mode_split(self):
  with tempfile.TemporaryDirectory() as temp:
   p=Path(temp);log=p/'source.jsonl';d=data();d['log_path']=str(log);d['framing_counters']={'crc_valid_packets':3}
   with patch('demo_metrics.time.monotonic',return_value=100):self.m.start_recording(d,'normal')
   self.m.observe(d,{},'normal',now=100)
   self.m.observe(data(stat(2,60,60),lat(),at=101),{},'normal',now=101)
   self.m.observe(data(stat(3,90,90)),{},'raw',waiting=True,now=102)
   records=[]
   for at,event,value in [(99,'latency',lat(1)),(100.1,'status_health',stat(1)),(101,'latency',lat(2)),(101.1,'latency',lat(2)),(101.2,'status_health',stat(2,60,60)),(101.3,'latency',lat(4,flags=17)),(102.1,'latency',lat(3))]:
    records.append(dict(session_id='test-session',monotonic_s=at,event=event,**{'packet' if event=='latency' else 'status':value}))
   log.write_text(''.join(json.dumps(r)+'\n' for r in records))
   d['framing_counters']={'crc_valid_packets':10,'crc_failures':1}
   with patch('demo_metrics.time.monotonic',return_value=103):out=self.m.export(p,d)
   report=json.loads((out/'summary.json').read_text())
   self.assertEqual(report['segments'][0]['accepted_samples'],2)
   self.assertEqual(report['segments'][0]['rejected_samples'],1)
   self.assertEqual(report['segments'][0]['board_tick_rates']['observed_new_fps'],30)
   self.assertFalse(report['segments'][1]['measurement_trust']['passed'])
   self.assertFalse(report['host_framing_window']['passed'])
   self.assertFalse(report['segments'][0]['measurement_trust']['passed'])
   self.assertEqual(len((out/'session_window.jsonl').read_text().splitlines()),6)
   self.assertIsNone(report['segments'][0]['distributions_us'])
   self.assertEqual(len(report['segments'][0]['distributions_us_by_config_and_actual_LAT_route']),2)
   self.assertTrue((out/'latency_samples.csv').exists())
   with self.assertRaises(ValueError):self.m.export(p)

unittest.main(argv=[sys.argv[0]],verbosity=2)
