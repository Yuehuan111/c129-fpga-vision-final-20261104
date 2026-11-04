"""Offline image inspection. No capture, serial, firmware write or build actions."""
import argparse,hashlib,json
from pathlib import Path
import cv2
import numpy as np
import measure_quality as quality


def read(path):
    im=cv2.imdecode(np.fromfile(path,np.uint8),cv2.IMREAD_COLOR)
    if im is None:raise ValueError(f'Cannot load {path}')
    return im


def corners(im,cols,rows):
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ok,c=cv2.findChessboardCornersSB(gray,(cols,rows),flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE)
    method='SB'
    if not ok:
        ok,c=cv2.findChessboardCorners(gray,(cols,rows),cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_ADAPTIVE_THRESH)
        if not ok:raise ValueError('Complete checkerboard not detected')
        c=cv2.cornerSubPix(gray,c,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,80,.001))
        method='classic+subpixel'
    return c.reshape(rows,cols,2).astype(float),method


def write_json(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def checker(a):
    image=read(a.image);c,method=corners(image,a.cols,a.rows)
    row_lines=[quality.analyze_line(line) for line in c]
    col_lines=[quality.analyze_line(line) for line in c.transpose(1,0,2)]
    hx=np.linalg.norm(np.diff(c,axis=1),axis=2);vy=np.linalg.norm(np.diff(c,axis=0),axis=2)
    summary={'measurement':'checkerboard_geometry','source':str(a.image.resolve()),
             'source_sha256':hashlib.sha256(a.image.read_bytes()).hexdigest(),
             'image_size':[image.shape[1],image.shape[0]],'inner_corners':[a.cols,a.rows],
             'detector':method,'point_count':a.cols*a.rows,'row_lines':row_lines,'column_lines':col_lines,
             'horizontal_spacing_mean_px':float(hx.mean()),'vertical_spacing_mean_px':float(vy.mean()),
             'horizontal_spacing_cv_percent':float(100*hx.std()/hx.mean()),
             'vertical_spacing_cv_percent':float(100*vy.std()/vy.mean()),
             'line_rms_max_px':max(x['rms_residual'] for x in row_lines+col_lines),
             'note':'Line straightness is not equal-spacing. Oblique perspective legitimately changes spacing. Corner localization/board flatness affect residuals.'}
    write_json(a.out.with_suffix('.json'),summary)
    vis=image.copy();cv2.drawChessboardCorners(vis,(a.cols,a.rows),c.astype(np.float32).reshape(-1,1,2),True)
    a.out.with_suffix('.png').write_bytes(cv2.imencode('.png',vis)[1].tobytes())
    print(json.dumps({k:v for k,v in summary.items() if k not in ['row_lines','column_lines']},indent=2))


def edge(a):
    roi=a.roi
    if a.pick:
        im=read(a.image);scale=min(1,1280/im.shape[1],800/im.shape[0])
        display=cv2.resize(im,None,fx=scale,fy=scale) if scale<1 else im
        r=cv2.selectROI('Select ONE edge, excluding corners; Enter confirms',display,showCrosshair=True)
        cv2.destroyAllWindows()
        if min(r[2:])<=0:raise ValueError('Selection cancelled')
        # Derive scales from actual rounded raster dimensions.
        sx,sy=im.shape[1]/display.shape[1],im.shape[0]/display.shape[0]
        x,y=int(np.floor(r[0]*sx)),int(np.floor(r[1]*sy))
        right,bottom=int(np.ceil((r[0]+r[2])*sx)),int(np.ceil((r[1]+r[3])*sy))
        roi=[x,y,min(right,im.shape[1])-x,min(bottom,im.shape[0])-y]
    if roi is None:raise ValueError('Provide --roi x y width height, or --pick')
    gray=quality.load_gray_roi(a.image,roi,a.transfer,a.gamma)
    result,curves=quality.analyze_edge(gray,a.axis)
    result.update(image=str(a.image.resolve()),roi=roi,source=a.source,
                  image_sha256=hashlib.sha256(a.image.read_bytes()).hexdigest(),
                  transfer=a.transfer,gamma=a.gamma if a.transfer=='gamma' else None)
    quality.save_edge_outputs(a.out,result,curves)
    print(json.dumps(result,indent=2))


def line_pick(a):
    im=read(a.image);scale=min(1,1280/im.shape[1],800/im.shape[0])
    display=cv2.resize(im,None,fx=scale,fy=scale) if scale<1 else im.copy()
    points=[];window='Click points on SAME line; Enter measures, Backspace undoes, Esc cancels'
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            points.append([x*im.shape[1]/display.shape[1],y*im.shape[0]/display.shape[0]])
    cv2.namedWindow(window);cv2.setMouseCallback(window,click)
    while True:
        shown=display.copy()
        for x,y in points:
            cv2.circle(shown,(round(x*display.shape[1]/im.shape[1]),round(y*display.shape[0]/im.shape[0])),3,(0,0,255),-1)
        cv2.imshow(window,shown);key=cv2.waitKey(30)&255
        if key in (10,13):break
        if key in (8,127) and points:points.pop()
        if key==27 or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:
            cv2.destroyAllWindows();raise ValueError('Selection cancelled')
    cv2.destroyAllWindows()
    pts=np.asarray(points,float);result=quality.analyze_line(pts)
    result.update(image=str(a.image.resolve()),source='manual-points-on-image',note='Manual click precision limits this measurement; same physical line required.')
    quality.save_line_outputs(a.out,result,pts)
    np.savetxt(a.out.parent/(a.out.name+'_points.csv'),pts,delimiter=',',header='x,y',comments='')
    print(json.dumps(result,indent=2))


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='command',required=True)
    s=sub.add_parser('checker');s.add_argument('--image',type=Path,required=True);s.add_argument('--cols',type=int,default=10);s.add_argument('--rows',type=int,default=5);s.add_argument('--out',type=Path,required=True)
    e=sub.add_parser('edge');e.add_argument('--image',type=Path,required=True)
    choice=e.add_mutually_exclusive_group(required=True);choice.add_argument('--roi',nargs=4,type=int);choice.add_argument('--pick',action='store_true')
    e.add_argument('--axis',choices=['vertical','horizontal'],required=True);e.add_argument('--transfer',choices=['linear','srgb','gamma'],required=True)
    e.add_argument('--gamma',type=float,default=2.2);e.add_argument('--source',choices=['real-chart','synthetic-test'],required=True);e.add_argument('--out',type=Path,required=True)
    l=sub.add_parser('line-pick');l.add_argument('--image',type=Path,required=True);l.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    if a.out.suffix:ap.error('--out must be a prefix without extension')
    a.out.parent.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(2)
    if a.command=='checker':checker(a)
    elif a.command=='edge':edge(a)
    elif a.command=='line-pick':line_pick(a)

if __name__=='__main__':main()
