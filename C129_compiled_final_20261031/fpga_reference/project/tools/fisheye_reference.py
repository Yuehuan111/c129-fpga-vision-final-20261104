"""Offline integer oracle for the synthesizable PDS fisheye mapper.

This never processes the board's live video or uploads a coordinate table.
Only Verilog generates maps on the FPGA. Parameters are Q16.16; division,
integer square root and CORDIC follow the bounded hardware arithmetic.
"""
import math
import numpy as np

MODEL = 'opencv_fisheye_k1_k2_k3_k4'
MODEL_TAG = 0x46495348
from project_profile import BUILD_ID
ORDER = ['fx_src','fy_src','cx_src','cy_src','cx_out','cy_out','fx_out','fy_out',
         'fish_k1','fish_k2','fish_k3','fish_k4','cos','sin','gain_q8','offset','mode','model_tag']
ANGLES = tuple(int(math.floor(math.atan(2.0**-i)*(1<<30)+.5)) for i in range(30))
Q = 1 << 28


def validate_words(words):
    if len(words) != 18 or words[17] != MODEL_TAG or words[16] not in (0,1):
        raise ValueError('Fisheye needs18 complete words, FISH model tag and mode0/1.')
    if any(isinstance(x,bool) or not isinstance(x,(int,np.integer)) or not -(1<<31)<=int(x)<(1<<31) for x in words):
        raise ValueError('Every register must be a signed32-bit integer.')
    if min(words[0],words[1]) <= 0 or min(words[6],words[7]) < 32*65536:
        raise ValueError('Source focal must be positive; output focal must be >=32 pixels.')
    if any(abs(x)>65536 for x in words[8:12]):
        raise ValueError('Supported fish coefficients lie between -1 and1.')
    if any(abs(x)>65536 for x in words[12:14]):
        raise ValueError('Rotation cos/sin must lie between -1 and1.')
    if not 0<=words[14]<=2048 or not -255<=words[15]<=255:
        raise ValueError('Brightness gain/offset outside supported range.')
    return list(map(int,words))


def vector_fixed_map(width,height,words,chunk_rows=32):
    p=validate_words(words)
    if not 1<=width<=2048 or not 1<=height<=2048:
        raise ValueError('Each image dimension must be1..2048.')
    out=np.zeros((height,width),dtype=np.uint64)
    error_count=0
    if p[16]==0:
        yy,xx=np.mgrid[:height,:width]
        return (np.uint64(1<<38)|(xx.astype(np.uint64)<<27)|(yy.astype(np.uint64)<<8)),0
    invx=(1<<56)//p[6];invy=(1<<56)//p[7]
    coeff=[v<<12 for v in p[8:12]]
    for first in range(0,height,chunk_rows):
        v,u=np.mgrid[first:min(first+chunk_rows,height),:width]
        du=(u.astype(np.int64)<<16)-p[4];dv=(v.astype(np.int64)<<16)-p[5]
        # These public parameter limits keep all numpy int64 intermediates in
        # range; hardware's96-bit multiplier has additional overflow checking.
        if max(int(np.max(np.abs(du))),int(np.max(np.abs(dv)))) >= (1<<27):
            raise ValueError('Output principal point leaves supported normalized domain.')
        x=(du*invx)>>28;y=(dv*invy)>>28
        x,y=((x*p[12])>>16)-((y*p[13])>>16),((x*p[13])>>16)+((y*p[12])>>16)
        bad=(np.abs(x)>4*Q)|(np.abs(y)>4*Q)
        # Domain errors are invalid pixels and invalidate publication in RTL.
        x=np.where(bad,0,x);y=np.where(bad,0,y)
        squared=x*x+y*y
        radius=np.sqrt(squared.astype(np.float64)).astype(np.int64)
        radius-=radius*radius>squared
        radius+=(radius+1)*(radius+1)<=squared
        vx=np.full_like(radius,Q);vy=radius.copy();angle=np.zeros_like(radius)
        for i,constant in enumerate(ANGLES):
            sx=vx>>i;sy=vy>>i;positive=vy>0;negative=vy<0
            vx=np.where(positive,vx+sy,np.where(negative,vx-sy,vx))
            vy=np.where(positive,vy-sx,np.where(negative,vy+sx,vy))
            angle=np.where(positive,angle+constant,np.where(negative,angle-constant,angle))
        theta=angle>>2;t2=(theta*theta)>>28
        h=np.full_like(theta,coeff[3])
        for k in (coeff[2],coeff[1],coeff[0],Q):h=((h*t2)>>28)+k
        td=(theta*h)>>28
        bad|=(td<0)|(td>0x7fffffff)
        safe_td=np.where(bad,0,td)
        scale=np.floor_divide(safe_td<<28,np.maximum(radius,1))
        scale=np.where(radius==0,Q,scale)
        xd=(x*scale)>>28;yd=(y*scale)>>28
        source_u=((p[0]*xd)>>16)+(p[2]<<12)
        source_v=((p[1]*yd)>>16)+(p[3]<<12)
        valid=(~bad)&(source_u>=0)&(source_v>=0)&(source_u<=(width-1)*Q)&(source_v<=(height-1)*Q)
        xq=np.where(valid,source_u>>20,0).astype(np.uint64)
        yq=np.where(valid,source_v>>20,0).astype(np.uint64)
        out[first:first+len(v)]=(valid.astype(np.uint64)<<38)|(xq<<19)|yq
        error_count+=int(bad.sum())
    return out,error_count


def float_map(width,height,words):
    p=np.asarray(validate_words(words),np.float64)
    v,u=np.mgrid[:height,:width]
    if int(words[16])==0:return u.astype(float),v.astype(float)
    a=(u-p[4]/65536)/(p[6]/65536);b=(v-p[5]/65536)/(p[7]/65536)
    a,b=a*p[12]/65536-b*p[13]/65536,a*p[13]/65536+b*p[12]/65536
    r=np.hypot(a,b);theta=np.arctan(r);t2=theta*theta
    k=p[8:12]/65536
    td=theta*(1+t2*(k[0]+t2*(k[1]+t2*(k[2]+t2*k[3]))))
    scale=np.divide(td,r,out=np.ones_like(r),where=r!=0)
    return (p[0]*a*scale+p[2])/65536,(p[1]*b*scale+p[3])/65536
