"""Independent integer reference for the RTL numerical contract (synthetic tests).

Q16.16 multiply truncates toward -infinity. Signed overflow makes a map invalid.
Output coordinates are unsigned Q11.8, RGB565 samples expand by bit replication.
"""
import math
import numpy as np

Q=65536
ORDER=['fx_src','fy_src','cx_src','cy_src','cx_out','cy_out','inv_fx_out','inv_fy_out',
       'k1','k2','p1','p2','cos','sin','gain_q8','offset']
def q16(v): return int(math.floor(float(v)*Q+.5))
def check(v):
    if not -(1<<31)<=v<(1<<31): raise OverflowError
    return v
def a(x,y): return check(x+y)
def m(x,y): return check((x*y)//Q)
def fixed_map(w,h,c):
    result=np.zeros((h,w),dtype=np.uint64); errors=0
    for v in range(h):
        for u in range(w):
            try:
                x=m(check(u*Q-c[4]),c[6]); y=m(check(v*Q-c[5]),c[7])
                rcx=m(x,c[12]); rsy=m(y,c[13]); rsx=m(x,c[13]); rcy=m(y,c[12])
                x=check(rcx-rsy);y=a(rsx,rcy)
                x2=m(x,x);y2=m(y,y);xy=m(x,y);r2=a(x2,y2);r4=m(r2,r2)
                k1r=m(c[8],r2);k2r=m(c[9],r4);rad=a(a(Q,k1r),k2r)
                xr=m(x,rad);yr=m(y,rad);p1xy=m(c[10],xy);p2xy=m(c[11],xy)
                tx=a(r2,a(x2,x2));ty=a(r2,a(y2,y2))
                p2tx=m(c[11],tx);p1ty=m(c[10],ty)
                xd=a(a(xr,a(p1xy,p1xy)),p2tx);yd=a(a(yr,p1ty),a(p2xy,p2xy))
                sx=a(m(c[0],xd),c[2]);sy=a(m(c[1],yd),c[3])
                if 0<=sx<=(w-1)*Q and 0<=sy<=(h-1)*Q:
                    xq=(sx+128)>>8;yq=(sy+128)>>8
                    if xq<=(w-1)*256 and yq<=(h-1)*256:
                        result[v,u]=(1<<38)|(xq<<19)|yq
            except OverflowError: errors+=1
    return result, errors
def float_map(w,h,c):
    p=np.asarray(c[:14],dtype=np.float64)/Q
    v,u=np.mgrid[:h,:w]
    x=(u-p[4])*p[6];y=(v-p[5])*p[7]
    x,y=x*p[12]-y*p[13],x*p[13]+y*p[12]
    r2=x*x+y*y;rad=1+p[8]*r2+p[9]*r2*r2
    return p[0]*(x*rad+2*p[10]*x*y+p[11]*(r2+2*x*x))+p[2],p[1]*(y*rad+p[10]*(r2+2*y*y)+2*p[11]*x*y)+p[3]
def rgb565(rgb):
    p=rgb.astype(np.uint16)
    return ((p[...,0]>>3)<<11)|((p[...,1]>>2)<<5)|(p[...,2]>>3)
def expand(p):
    p=p.astype(np.uint16);r=(p>>11)&31;g=(p>>5)&63;b=p&31
    return np.stack([(r<<3)|(r>>2),(g<<2)|(g>>4),(b<<3)|(b>>2)],axis=-1).astype(np.uint8)
def sample(src,entries,c):
    h,w=src.shape; p=expand(src).astype(np.int64);e=entries.astype(np.int64)
    valid=(e>>38)!=0;xq=(e>>19)&((1<<19)-1);yq=e&((1<<19)-1)
    x0=xq>>8;y0=yq>>8;x1=np.minimum(x0+1,w-1);y1=np.minimum(y0+1,h-1)
    fx=(xq&255)[...,None];fy=(yq&255)[...,None]
    value=(p[y0,x0]*(256-fx)*(256-fy)+p[y0,x1]*fx*(256-fy)+p[y1,x0]*(256-fx)*fy+p[y1,x1]*fx*fy+32768)>>16
    value=np.clip(((value*c[14]+128)>>8)+c[15],0,255).astype(np.uint8)
    value[~valid]=0
    return rgb565(value)
def params(w,h,angle=0,distort=True):
    focal=w*.65;cx=(w-1)/2;cy=(h-1)/2
    p=[focal,focal,cx,cy,cx,cy,1/focal,1/focal,-.32 if distort else 0,.09 if distort else 0,.003 if distort else 0,-.002 if distort else 0,math.cos(math.radians(angle)),math.sin(math.radians(angle))]
    return [q16(v) for v in p]+[256,0]
