"""Physical720p checks: CRC status,18-word control, unmodified HDMI capture.
No PC-corrected image or precomputed FPGA map is sent by this tool.
"""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from project_profile import ROOT, BUILD_ID, DEFAULT_CALIBRATION


def save(out, name, value):
    (out/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def status(port_name, seconds, out, label):
    import serial
    from serial.tools import list_ports
    from sensor_register import verify_cp210x
    from read_board_status import extract_packets, add_frame_rates
    verify_cp210x(port_name, list_ports.comports())
    records=[];buffer=bytearray();previous=None
    with serial.Serial(port_name,115200,timeout=.2) as port:
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            buffer.extend(port.read(1024))
            for item in extract_packets(buffer):
                if item['build_id']!=BUILD_ID:
                    raise ValueError(f'Wrong firmware:0x{item["build_id"]:08X}')
                add_frame_rates(item, previous);previous=item.copy();records.append(item)
    save(out, label+'_status.json', records)
    if len(records)<3:
        raise RuntimeError('Fewer than3 valid status packets; no physical FPS established')
    summary=records[-1].copy();add_frame_rates(summary,records[0])
    summary.update(host_time=datetime.now().astimezone().isoformat(),status_packets=len(records),
                   first_board_tick=records[0]['sample_ticks'],last_board_tick=records[-1]['sample_ticks'])
    for name in ('captured','processed','raw_valid_frames','underflow','overflow','dropped','camera_skipped_frames'):
        if name in records[0] and name in records[-1]:
            summary['delta_'+name]=(records[-1][name]-records[0][name])&0xFFFFFFFF
    save(out,label+'_summary.json',summary)
    print(json.dumps({k:summary.get(k) for k in ('build_id','status_packets','sample_interval_seconds',
        'observed_raw_fps','observed_new_fps','observed_pclk_hz','map_building','camera_error',
        'ddr_initialized','error_sticky','delta_underflow','delta_overflow','delta_dropped',
        'last_frame_service_fps','last_line_pixels','vsync_rise_lines','vsync_fall_lines')},ensure_ascii=False))
    return summary


def capture(out,label,device):
    import cv2
    from pygrabber.dshow_graph import FilterGraph
    devices=FilterGraph().get_input_devices()
    if device>=len(devices) or devices[device]!='USB Video':
        raise ValueError('Selected device is not the identified USB Video HDMI capture card')
    cap=cv2.VideoCapture(device,cv2.CAP_DSHOW)
    try:
        if not cap.isOpened():raise RuntimeError('HDMI capture cannot open')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        cap.set(cv2.CAP_PROP_FPS,30)
        frames=[];started=time.monotonic()
        for i in range(120):
            ok,image=cap.read()
            if not ok:raise RuntimeError('No HDMI capture frame')
            if image.shape[:2]!=(720,1280):raise ValueError('Capture did not return1280x720')
            if i in (59,89,119):
                file=out/f'{label}_{i}.png';ok,encoded=cv2.imencode('.png',image)
                if not ok:raise RuntimeError('Cannot encode capture')
                encoded.tofile(file);frames.append({'file':str(file),'size':[1280,720],
                    'mean_bgr':image.mean(axis=(0,1)).tolist(),'stddev':float(image.std())})
        result={'host_time':datetime.now().astimezone().isoformat(),'device':devices[device],
            'reported_usb_fps':cap.get(cv2.CAP_PROP_FPS),'elapsed':time.monotonic()-started,
            'frames':frames,'PC_geometric_correction_applied':False,
            'note':'USB capture dimensions/rate alone do not certify sensor geometry or new-result FPS.'}
        save(out,label+'_capture.json',result);print(json.dumps(result,ensure_ascii=False))
    finally:cap.release()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action',choices=['status','capture','raw','corrected'])
    ap.add_argument('--port',default='COM15');ap.add_argument('--device',type=int,default=0)
    ap.add_argument('--seconds',type=int,default=12,choices=range(4,19))
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--label',required=True)
    args=ap.parse_args()
    if not args.label.replace('_','').isalnum():raise ValueError('Invalid label')
    args.out.mkdir(parents=True,exist_ok=True)
    if args.action=='status':status(args.port,args.seconds,args.out,args.label)
    elif args.action=='capture':capture(args.out,args.label,args.device)
    else:
        from host_actions import FISHEYE_IDENTITY, configuration, load_calibration, fitted_configuration, send_configuration
        if args.action=='raw':
            words=configuration(FISHEYE_IDENTITY);metadata={'width':1280,'height':720,
                'source':'identity','synthetic':True,'identity':True,'angle':0,'zoom':1}
        else:
            words,metadata=fitted_configuration(load_calibration(DEFAULT_CALIBRATION),zoom=1,angle=0,auto_crop=False)
            metadata.update(source=str(DEFAULT_CALIBRATION),synthetic=False,identity=False)
        message=send_configuration(args.port,words,metadata)
        confirmation=json.loads((ROOT/'output/last_config/status_confirmation.json').read_text(encoding='utf-8'))
        save(args.out,args.label+'_parameters.json',{'words':words,'metadata':metadata,
            'message':message,'confirmation':confirmation})
        print(message)


if __name__=='__main__':main()
