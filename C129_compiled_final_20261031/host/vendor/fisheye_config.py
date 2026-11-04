"""Fisheye register parameters for the PDS Verilog implementation.

No coordinate data is sent.18 registers+COMMIT; advanced build ID0720A001.
"""
import math
from fisheye_reference import MODEL,MODEL_TAG,ORDER,validate_words

IDENTITY={'model':MODEL,'width':1280,'height':720,'synthetic':True,'identity_mode':True,
          'camera_matrix':[[1024.,0.,639.5],[0.,1024.,359.5],[0.,0.,1.]],
          'output_camera_matrix':[[1024.,0.,639.5],[0.,1024.,359.5],[0.,0.,1.]],
          'distortion':[0.,0.,0.,0.]}


def normalize(d):
    if d.get('model')!=MODEL:raise ValueError('Expected OpenCV fisheye model.')
    if 'input_image_size' in d and isinstance(d.get('parameters'),dict):
        if d.get('dist_coeff_order')!=['fish_k1','fish_k2','fish_k3','fish_k4']:
            raise ValueError('Unexpected fisheye coefficient order.')
        if d.get('fractional_bits')!=16 or d.get('total_bits')!=32 or d.get('all_parameters_fit') is not True:
            raise ValueError('C parameter format must be signed32 Q16.16 without overflow.')
        def get(group,key):
            item=group[key];value=item['decimal']
            if isinstance(value,bool) or not isinstance(value,int) or not -(1<<31)<=value<(1<<31):
                raise ValueError('Invalid signed parameter word.')
            restored=value/65536
            if item['restored_float']!=restored:raise ValueError('Parameter decimal/restored value mismatch.')
            return restored
        p=d['parameters'];o=d['output_parameters']
        width,height=d['input_image_size']
        if d.get('output_image_size')!=[width,height]:raise ValueError('Input/output dimensions must match.')
        d={**d,'width':width,'height':height,'synthetic':False,'identity_mode':False,
                'camera_matrix':[[get(p,'fx'),0,get(p,'cx')],[0,get(p,'fy'),get(p,'cy')],[0,0,1]],
                'output_camera_matrix':[[get(o,'fx_out'),0,get(o,'cx_out')],[0,get(o,'fy_out'),get(o,'cy_out')],[0,0,1]],
                'distortion':[get(p,'fish_k'+str(i)) for i in range(1,5)]}
    if 'image_size' in d:
        if d.get('output_image_size')!=d['image_size']:raise ValueError('Input/output dimensions must match.')
        if d.get('dist_coeff_order')!=['fish_k1','fish_k2','fish_k3','fish_k4']:
            raise ValueError('Unexpected fisheye coefficient order.')
        d={**d,'width':d['image_size'][0],'height':d['image_size'][1],
           'distortion':d['dist_coeffs'],'synthetic':False,'identity_mode':False}
    if d.get('roi_cropped',False) is not False:raise ValueError('Cropped input requires explicit coordinate conversion.')
    if d.get('rectification_rotation',[[1,0,0],[0,1,0],[0,0,1]])!=[[1,0,0],[0,1,0],[0,0,1]]:
        raise ValueError('Only identity3D rectification is implemented;2D roll has separate registers.')
    return d


def configuration(d,angle=0,zoom=1,gain=1,offset=0,shift_x=0,shift_y=0):
    d=normalize(d)
    def number(value):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError('Every camera setting must be a finite number.')
        return float(value)
    w,h=d.get('width'),d.get('height')
    if any(isinstance(v,bool) or not isinstance(v,int) or not 1<=v<=2048 for v in (w,h)):
        raise ValueError('Integer image dimensions must be1..2048.')
    def matrix(value):
        if not isinstance(value,list) or len(value)!=3 or any(not isinstance(row,list) or len(row)!=3 for row in value):
            raise ValueError('Complete3x3 camera matrix required.')
        m=[[number(x) for x in row] for row in value]
        if any(abs(m[r][c])>1e-9 for r,c in ((0,1),(1,0),(2,0),(2,1))) or abs(m[2][2]-1)>1e-9:
            raise ValueError('Zero-skew standard camera matrix required.')
        return m
    k=matrix(d.get('camera_matrix'));ko=matrix(d.get('output_camera_matrix'))
    dist=d.get('distortion')
    if not isinstance(dist,list) or len(dist)!=4:raise ValueError('Use exactly four fisheye coefficients.')
    dist=list(map(number,dist));angle,zoom,gain,offset,shift_x,shift_y=map(number,(angle,zoom,gain,offset,shift_x,shift_y))
    if not .25<=zoom<=8 or not 0<=gain<=8 or not -255<=offset<=255 or not offset.is_integer():
        raise ValueError('Invalid zoom/brightness setting.')
    rad=math.radians(angle)
    values=[k[0][0],k[1][1],k[0][2],k[1][2],ko[0][2]+shift_x,ko[1][2]+shift_y,
            ko[0][0]*zoom,ko[1][1]*zoom,*dist,math.cos(rad),math.sin(rad)]
    words=[int(math.floor(v*65536+.5)) for v in values]+[int(math.floor(gain*256+.5)),int(offset),0 if d.get('identity_mode') else 1,MODEL_TAG]
    return validate_words(words)
