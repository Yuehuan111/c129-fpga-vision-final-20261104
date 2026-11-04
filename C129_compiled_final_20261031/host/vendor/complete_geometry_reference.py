"""Offline bit-exact oracle for complete_fisheye_mapper (never uploaded as a map).

Only the 18 calibration words and nine matrix words may be sent to the FPGA.
This module duplicates the FPGA integer arithmetic for validation/ROI admission.
"""
from __future__ import annotations
from functools import lru_cache
from math import isqrt
import numpy as np
from project_profile import TANGENTIAL_Q16

CORDIC=(843314857,497837829,263043837,133525159,67021687,33543516,
16775851,8388437,4194283,2097149,1048576,524288,262144,131072,
65536,32768,16384,8192,4096,2048,1024,512,256,128,64,32,16,8,4,2)
IDENTITY=(65536,0,0,0,65536,0,0,0,65536)

def signed32(x):
    return (int(x)+2**31)%2**32-2**31

def atan_q30(radius):
    x,y,a=1<<28,int(radius),0
    for i,step in enumerate(CORDIC):
        sx,sy=x>>i,y>>i
        if y>0:x,y,a=x+sy,y-sx,a+step
        elif y<0:x,y,a=x-sy,y+sx,a-step
    return a

@lru_cache(maxsize=16)
def radial_table(coefficients):
    k1,k2,k3,k4=(signed32(x)<<12 for x in coefficients)
    data=np.zeros(4097,dtype=np.int64)
    valid=np.zeros(4097,dtype=bool)
    data[0]=1<<30;valid[0]=True
    for i in range(1,4097):
        r=isqrt(i<<49);theta=atan_q30(r)>>2;t2=(theta*theta)>>28
        p=(k4*t2)>>28
        p=((p+k3)*t2)>>28
        p=((p+k2)*t2)>>28
        p=((p+k1)*t2)>>28
        td=((p+(1<<28))*theta)>>28
        if 0<=td<=2147483647:
            scale=(td<<28)//r
            if scale<536870912:data[i]=scale<<2;valid[i]=True
    data.setflags(write=False);valid.setflags(write=False)
    return data,valid

def compose(words18,matrix9):
    w=[signed32(x) for x in words18];h=[signed32(x) for x in matrix9]
    if len(w)!=18 or len(h)!=9:raise ValueError('18 calibration + 9 matrix words required')
    if w[16] not in (0,1):raise ValueError('mode must be 0 or 1')
    if w[16]==0:return w,h,None,None,None
    if not(0<w[0]<=268435456 and 0<w[1]<=268435456 and w[6]>=2097152 and w[7]>=2097152):
        raise ValueError('focal length outside RTL domain')
    if any(abs(x)>65536 for x in w[12:14]) or any(abs(x)>524288 for x in h+w[8:12]):
        raise ValueError('rotation or matrix coefficient outside RTL domain')
    if max(abs(x) for x in h)<32768:
        raise ValueError('matrix scale too small: max(abs(H)) must be >= 0.5')
    invx,invy=(1<<56)//w[6],(1<<56)//w[7]
    dx=[];dy=[];origin=[]
    for row in range(3):
        a,b,c=h[row*3:row*3+3]
        ux=(a*w[12]+b*w[13])>>16
        vy=(b*w[12]-a*w[13])>>16
        ddx=(ux*invx)>>24;ddy=(vy*invy)>>24
        o=(c<<16)-((ddx*w[4])>>16)-((ddy*w[5])>>16)
        if not all(-(1<<47)<=x<(1<<47) for x in (ddx,ddy,o)):
            raise ValueError('homogeneous setup overflow')
        dx.append(ddx);dy.append(ddy);origin.append(o)
    return w,h,dx,dy,origin

def tangential_q28(x,y,tangential_q16=TANGENTIAL_Q16):
    """Apply post-fisheye tangential distortion, with one floor per Q28 delta.

    Products are exact signed integers. Quotient/remainder decomposition avoids
    overflowing NumPy int64 when coefficient-weighted Q56 products exceed it.
    Returns (x, y, overflow); no valid result is saturated or wrapped to 32 bits.
    The p=0 build branch intentionally returns its input without changing it.
    """
    p1,p2=(int(p) for p in tangential_q16)
    if any(not -(1<<15)<=p<(1<<15) for p in (p1,p2)):
        raise ValueError('Tangential build coefficients must fit signed Q0.16.')
    x,y=np.broadcast_arrays(np.asarray(x,dtype=np.int64),np.asarray(y,dtype=np.int64))
    if p1==0 and p2==0:return x,y,np.zeros(x.shape,dtype=bool)
    bad=(x<-(1<<31))|(x>=(1<<31))|(y<-(1<<31))|(y>=(1<<31))
    sx=np.where(bad,0,x);sy=np.where(bad,0,y)
    xx,yy,xy=sx*sx,sy*sy,sx*sy
    mask=(1<<44)-1
    def weighted_floor(coefficients):
        # a = (a >> 44)*2**44 + (a & mask), also for negative xy.
        products=(xx,yy,xy)
        high=sum(c*(a>>44) for c,a in zip(coefficients,products))
        low=sum(c*(a&mask) for c,a in zip(coefficients,products))
        return high+(low>>44)
    tx=sx+weighted_floor((3*p2,p2,2*p1))
    ty=sy+weighted_floor((p1,3*p1,2*p2))
    bad|=(tx<-(1<<31))|(tx>=(1<<31))|(ty<-(1<<31))|(ty>=(1<<31))
    return tx,ty,bad


def vector_fixed_map(width,height,words18,matrix9=IDENTITY,*,tangential_q16=TANGENTIAL_Q16):
    """Return (uint64[height,width] map entries, domain_invalid_count).

    Raises ValueError for a configuration that RTL rejects. Domain invalid pixels
    include nonforward rays, numerical domain limits, and invalid radial samples.
    Ordinary source-image borders are excluded from domain_invalid_count.
    """
    w,h,dx,dy,origin=compose(words18,matrix9)
    yy,xx=np.mgrid[:height,:width].astype(np.int64)
    if w[16]==0:return ((np.uint64(1)<<np.uint64(38))|((xx.astype(np.uint64)<<np.uint64(8))<<np.uint64(19))|(yy.astype(np.uint64)<<np.uint64(8))),0
    rays=[o+x*xx+y*yy for o,x,y in zip(origin,dx,dy)]
    X,Y,Z=rays
    bad=(Z<(1<<26))  # Near-singular/behind rays: require homogeneous Z >= 1/64.
    for r in rays:bad|=(r<-(1<<47))|(r>=(1<<47))
    safez=np.maximum(Z,1)
    lead=np.zeros(Z.shape,dtype=np.int64)
    # Integer leading-bit normalization exactly matches RTL (no float log2).
    for bit in range(47):lead=np.where(safez>=(1<<bit),bit,lead)
    def normalize(v,bits):
        return np.where(lead>=bits,v>>np.maximum(lead-bits,0),v<<np.maximum(bits-lead,0))
    nx,ny=normalize(X,28),normalize(Y,28)
    mantissa=normalize(Z,30)
    normlimit=np.left_shift(np.int64(1),lead+3)
    bad|=(X< -normlimit)|(X>=normlimit)|(Y< -normlimit)|(Y>=normlimit)
    # Clamp only poisoned intermediate data to avoid numpy overflow; it cannot
    # become valid. No valid output/source coordinate is ever clamped.
    nx=np.where(bad,0,nx);ny=np.where(bad,0,ny)
    mantissa=np.where(bad,1<<30,mantissa)
    index=(mantissa>>20)&1023
    seed=(1<<41)//(2049+2*index)
    correction=(1<<31)-((mantissa*seed)>>30)
    reciprocal=(seed*correction)>>30
    nx=(nx*reciprocal)>>30;ny=(ny*reciprocal)>>30
    bad|=(np.abs(nx)>(1<<30))|(np.abs(ny)>(1<<30))
    nx=np.where(bad,0,nx);ny=np.where(bad,0,ny)
    r2=nx*nx+ny*ny
    bad|=r2>=(1<<61)
    index=np.where(bad,0,r2>>49)
    fraction=(r2>>29)&((1<<20)-1)
    table,tvalid=radial_table(tuple(w[8:12]))
    bad|=~tvalid[index]|~tvalid[index+1]
    scale=table[index]+(((table[index+1]-table[index])*fraction)>>20)
    xd=(nx*scale)>>30;yd=(ny*scale)>>30
    xd,yd,tbad=tangential_q28(xd,yd,tangential_q16)
    bad|=tbad
    xd=np.where(bad,0,xd);yd=np.where(bad,0,yd)
    u=((xd*w[0])>>16)+(w[2]<<12)
    v=((yd*w[1])>>16)+(w[3]<<12)
    valid=(~bad)&(u>=0)&(v>=0)&(u<=((width-1)<<28))&(v<=((height-1)<<28))
    entries=np.where(valid,(np.uint64(1)<<np.uint64(38))|(((u+(1<<19))>>20).astype(np.uint64)<<np.uint64(19))|((v+(1<<19))>>20).astype(np.uint64),np.uint64(0))
    return entries,int(bad.sum())

def float_map(width,height,words18,matrix9=IDENTITY,*,tangential_q16=TANGENTIAL_Q16):
    """Independent float64 fisheye then tangential model, without quantized ops."""
    if signed32(words18[16])==0:
        yy,xx=np.mgrid[:height,:width]
        return xx.astype(np.float64),yy.astype(np.float64),np.ones((height,width),dtype=bool)
    w=np.array([signed32(x) for x in words18],dtype=np.float64)/65536
    h=np.array([signed32(x) for x in matrix9],dtype=np.float64).reshape(3,3)/65536
    yy,xx=np.mgrid[:height,:width]
    x=(xx-w[4])/w[6];y=(yy-w[5])/w[7]
    rx=x*w[12]-y*w[13];ry=x*w[13]+y*w[12]
    X=h[0,0]*rx+h[0,1]*ry+h[0,2]
    Y=h[1,0]*rx+h[1,1]*ry+h[1,2]
    Z=h[2,0]*rx+h[2,1]*ry+h[2,2]
    x=np.divide(X,Z,out=np.zeros_like(X),where=Z>0)
    y=np.divide(Y,Z,out=np.zeros_like(Y),where=Z>0)
    r=np.hypot(x,y);t=np.arctan(r);t2=t*t
    td=t*(1+t2*(w[8]+t2*(w[9]+t2*(w[10]+t2*w[11]))))
    s=np.divide(td,r,out=np.ones_like(r),where=r!=0)
    xd=x*s;yd=y*s
    p1,p2=(int(p)/65536 for p in tangential_q16)
    xx,yy,xy=xd*xd,yd*yd,xd*yd
    xt=xd+3*p2*xx+p2*yy+2*p1*xy
    yt=yd+p1*xx+3*p1*yy+2*p2*xy
    u=w[0]*xt+w[2];v=w[1]*yt+w[3]
    valid=(Z>=1/64)&(np.abs(x)<=4)&(np.abs(y)<=4)&(r*r<32)&(u>=0)&(u<=width-1)&(v>=0)&(v<=height-1)
    return u,v,valid
