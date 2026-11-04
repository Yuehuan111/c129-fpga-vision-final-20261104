"""Validate calibration JSON; export complete atomic FPGA settings and UART packets.

No serial transmission unless --port is explicitly given. Board transmission
uses CRC-checked build/version status; map publication does not confirm display.
Brown k1,k2,p1,p2 only: do not put OpenCV fisheye theta coefficients here.
"""
import argparse, json, math, struct, time
from pathlib import Path
from reference import ORDER, q16

def crc16(data):
    crc=0xffff
    for byte in data:
        crc ^= byte<<8
        for _ in range(8): crc=((crc<<1)^0x1021)&0xffff if crc&0x8000 else (crc<<1)&0xffff
    return crc

def packet(address,value):
    b=bytes([0xa5,address])+struct.pack('<I',value&0xffffffff)
    return b+struct.pack('>H',crc16(b))

def normalize_calibration(d):
    """Adapt the saved calibration schema without changing its optical model."""
    if not isinstance(d,dict): raise ValueError('Calibration must be an object')
    if d.get('model')=='opencv_fisheye_k1_k2_k3_k4':
        from fisheye_config import normalize
        return normalize(d)
    if d.get('model')!='pinhole_k1_k2_p1_p2_k3_fixed_zero': return d
    if d.get('dist_coeff_order')!=['k1','k2','p1','p2','k3']:
        raise ValueError('Unexpected Brown coefficient order')
    dist=d.get('dist_coeffs')
    if not isinstance(dist,list) or len(dist)!=5 or dist[4]!=0:
        raise ValueError('This hardware requires k3=0')
    size=d.get('image_size')
    if not isinstance(size,list) or len(size)!=2 or d.get('output_image_size')!=size:
        raise ValueError('Input/output sizes must match this hardware build')
    if d.get('roi_cropped') is not False:
        raise ValueError('Cropped calibration output requires an explicit coordinate conversion')
    if d.get('rectification_rotation')!=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]:
        raise ValueError('Nonidentity rectification rotation is not supported by this adapter')
    if 'output_camera_matrix' not in d: raise ValueError('Missing output camera matrix')
    return {**d,'source_model':d['model'],'model':'brown_k1_k2_p1_p2',
            'width':size[0],'height':size[1],'distortion':dist[:4],
            'fixed_k3':0,'synthetic':False,'validation_status':'candidate_requires_board_validation'}

def configuration(d,angle=0,zoom=1,gain=1,offset=0,shift_x=0,shift_y=0):
    d=normalize_calibration(d)
    def finite_number(value,label):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError(f'{label} must be a finite number')
        return float(value)
    if not isinstance(d,dict): raise ValueError('Calibration must be a JSON object')
    if d.get('model') == 'opencv_fisheye_k1_k2_k3_k4':
        from fisheye_config import configuration as fish_configuration
        return fish_configuration(d,angle,zoom,gain,offset,shift_x,shift_y)
    if d.get('model')!='brown_k1_k2_p1_p2': raise ValueError('Expected brown_k1_k2_p1_p2; theta fisheye coefficients are incompatible')
    w,h=d.get('width'),d.get('height')
    if any(isinstance(v,bool) or not isinstance(v,int) for v in (w,h)):
        raise ValueError('Image width and height must be JSON integers, without rounding or coercion')
    if not (1<=w<=2048 and 1<=h<=2048): raise ValueError('RTL coordinate limit: each dimension 1..2048')
    k=d.get('camera_matrix');dist=d.get('distortion')
    if not isinstance(k,list) or len(k)!=3 or any(not isinstance(row,list) or len(row)!=3 for row in k):
        raise ValueError('camera_matrix must be a complete 3x3 matrix')
    k=[[finite_number(value,f'camera_matrix[{r}][{c}]') for c,value in enumerate(row)] for r,row in enumerate(k)]
    if any(abs(k[r][c])>1e-9 for r,c in ((0,1),(1,0),(2,0),(2,1))) or abs(k[2][2]-1)>1e-9:
        raise ValueError('Only zero-skew standard camera matrices supported')
    if not isinstance(dist,list) or len(dist)!=4: raise ValueError('Use exactly [k1,k2,p1,p2]; calibrate with k3 fixed to zero')
    dist=[finite_number(value,f'distortion[{i}]') for i,value in enumerate(dist)]
    if finite_number(d.get('fixed_k3',0),'fixed_k3')!=0: raise ValueError('k3 must be fixed to zero; this RTL has no k3 term')
    angle=finite_number(angle,'angle');zoom=finite_number(zoom,'zoom');gain=finite_number(gain,'gain')
    offset_value=finite_number(offset,'offset')
    if not offset_value.is_integer(): raise ValueError('offset must be an integer')
    offset=int(offset_value)
    shift_x=finite_number(shift_x,'shift_x');shift_y=finite_number(shift_y,'shift_y')
    fx,fy,cx,cy=k[0][0],k[1][1],k[0][2],k[1][2]
    if not (fx>0 and fy>0 and .25<=zoom<=8 and 0<=gain<=8 and -255<=offset<=255): raise ValueError('Invalid focal/zoom/gain/offset')
    # A separate K_out must not be silently replaced by the input intrinsics.
    ko=d.get('output_camera_matrix')
    if ko is None:
        ofx,ofy,ocx,ocy=fx,fy,(w-1)/2,(h-1)/2
    else:
        if not isinstance(ko,list) or len(ko)!=3 or any(not isinstance(row,list) or len(row)!=3 for row in ko):
            raise ValueError('output_camera_matrix must be 3x3')
        ko=[[finite_number(v,'output_camera_matrix') for v in row] for row in ko]
        if any(abs(ko[r][c])>1e-9 for r,c in ((0,1),(1,0),(2,0),(2,1))) or abs(ko[2][2]-1)>1e-9:
            raise ValueError('Output camera matrix must be zero-skew standard form')
        ofx,ofy,ocx,ocy=ko[0][0],ko[1][1],ko[0][2],ko[1][2]
        if ofx<=0 or ofy<=0: raise ValueError('Invalid output focal length')
    rad=math.radians(angle)
    values=[fx,fy,cx,cy,ocx+shift_x,ocy+shift_y,1/(ofx*zoom),1/(ofy*zoom),*dist,math.cos(rad),math.sin(rad)]
    if not all(math.isfinite(x) for x in values): raise ValueError('Nonfinite parameter')
    words=[q16(x) for x in values]+[int(math.floor(gain*256+.5)),int(offset)]
    if not all(-(1<<31)<=v<(1<<31) for v in words): raise ValueError('Q16.16 overflow')
    if words[6]<=0 or words[7]<=0: raise ValueError('Output reciprocal focal quantized to zero')
    return words

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('calibration',type=Path);ap.add_argument('--out',type=Path,default=Path('output/config'))
    ap.add_argument('--angle',type=float,default=0);ap.add_argument('--zoom',type=float,default=1)
    ap.add_argument('--gain',type=float,default=1);ap.add_argument('--offset',type=int,default=0)
    ap.add_argument('--shift-x',type=float,default=0);ap.add_argument('--shift-y',type=float,default=0)
    ap.add_argument('--port',help='Explicit optional COM port. Requires pyserial and verified physical UART connection.')
    a=ap.parse_args();d=normalize_calibration(json.loads(a.calibration.read_text(encoding='utf-8-sig')))
    words=configuration(d,a.angle,a.zoom,a.gain,a.offset,a.shift_x,a.shift_y)
    metadata={'source':str(a.calibration),'synthetic':d.get('synthetic',False),
              'width':d['width'],'height':d['height'],'angle_degrees':a.angle,'zoom':a.zoom}
    if a.port:
        # Import only inside the CLI action: host_actions imports packet and
        # configuration from this module, so a top-level import would cycle.
        from host_actions import send_configuration, validate_board_configuration
        validate_board_configuration(words,metadata)
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/'config.hex').write_text(''.join(f'{v&0xffffffff:08x}\n' for v in words),encoding='ascii')
    packets=[packet(i,v) for i,v in enumerate(words)]+[packet(0xff,0x434f4d54)]
    (a.out/'uart_packets.bin').write_bytes(b''.join(packets))
    if len(words)==18:
        from fisheye_reference import ORDER as export_order
    else:export_order=ORDER
    (a.out/'config.json').write_text(json.dumps({**metadata,'model':d['model'],'words':dict(zip(export_order,words))},indent=2),encoding='utf-8')
    print(f'Exported {len(words)} words + COMMIT, {sum(map(len,packets))} UART bytes -> {a.out}')
    if a.port:
        print(send_configuration(a.port,words,metadata))

if __name__=='__main__': main()
