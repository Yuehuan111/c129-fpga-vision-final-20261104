"""Brown calibration from real checkerboard photographs, k3 fixed at zero.
The --cols/--rows are INNER corner counts. All images must match camera mode.
No automatic image resizing; save rejected views and per-view errors for review.
"""
import argparse,json,math
from pathlib import Path
import cv2
import numpy as np

def load(path): return cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
def save(path,img): path.write_bytes(cv2.imencode('.png',img)[1].tobytes())

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('images',type=Path);ap.add_argument('--cols',type=int,required=True);ap.add_argument('--rows',type=int,required=True)
    ap.add_argument('--square-mm',type=float,required=True);ap.add_argument('--out',type=Path,default=Path('output/calibration'))
    ap.add_argument('--synthetic',action='store_true',help='Mark generated test views; never present these as real calibration')
    ap.add_argument('--model',choices=('brown','fisheye'),default='brown')
    a=ap.parse_args()
    if min(a.cols,a.rows)<3 or a.square_mm<=0: raise ValueError('Invalid checkerboard dimensions')
    a.out.mkdir(parents=True,exist_ok=True);preview=a.out/'corners';preview.mkdir(exist_ok=True)
    obj=np.zeros((a.cols*a.rows,3),np.float32);obj[:,:2]=np.mgrid[:a.cols,:a.rows].T.reshape(-1,2)*a.square_mm
    objects=[];corners=[];accepted=[];rejected=[];size=None
    for path in sorted(a.images.iterdir()):
        if path.suffix.lower() not in ('.jpg','.jpeg','.png','.bmp','.tif','.tiff'):continue
        image=load(path)
        if image is None: rejected.append({'path':str(path),'reason':'unreadable'});continue
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);this_size=(gray.shape[1],gray.shape[0])
        if size is not None and size!=this_size: raise ValueError(f'Mixed resolutions: {path}; never silently resize calibration images')
        size=this_size
        ok,pts=cv2.findChessboardCornersSB(gray,(a.cols,a.rows),flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE)
        if not ok: rejected.append({'path':str(path),'reason':'corners not detected'});continue
        objects.append(obj.copy());corners.append(pts.astype(np.float32));accepted.append(str(path))
        cv2.drawChessboardCorners(image,(a.cols,a.rows),pts,ok);save(preview/(path.stem+'.png'),image)
    (a.out/'input_views.json').write_text(json.dumps({'accepted':accepted,'rejected':rejected},indent=2,ensure_ascii=False),encoding='utf-8')
    if len(objects)<10: raise ValueError(f'Only {len(objects)} accepted views. Need >=10 for this tool; aim 20-30 varied views covering edges. See input_views.json')
    if a.model=='fisheye':
        obj64=[np.ascontiguousarray(ob,np.float64).reshape(1,-1,3) for ob in objects]
        pts64=[np.ascontiguousarray(pt,np.float64).reshape(1,-1,2) for pt in corners]
        flags=(cv2.fisheye.CALIB_FIX_SKEW|cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC|
               cv2.fisheye.CALIB_CHECK_COND|cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)
        fits=[]
        for focal in (.55*max(size),.40*size[0]):
            initial=np.array([[focal,0,(size[0]-1)/2],[0,focal,(size[1]-1)/2],[0,0,1]],np.float64)
            try:
                fit=cv2.fisheye.calibrate(obj64,pts64,size,initial,np.zeros((4,1),np.float64),
                    flags=flags,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,200,1e-10))
                if np.isfinite(fit[0]) and np.isfinite(fit[1]).all() and np.isfinite(fit[2]).all():fits.append(fit)
            except cv2.error:pass
        if not fits:raise ValueError('Fisheye fitting failed for both initial focal guesses; no views were removed.')
        rms,K,D,rvecs,tvecs=min(fits,key=lambda fit:fit[0])
    else:rms,K,D,rvecs,tvecs=cv2.calibrateCamera(objects,corners,size,None,None,flags=cv2.CALIB_FIX_K3)
    errors=[]
    for path,ob,pt,rv,tv in zip(accepted,objects,corners,rvecs,tvecs):
        if a.model=='fisheye':predicted,_=cv2.fisheye.projectPoints(np.asarray(ob,np.float64).reshape(1,-1,3),rv,tv,K,D)
        else:predicted,_=cv2.projectPoints(ob,rv,tv,K,D)
        errors.append({'path':path,'rms_pixels':float(np.sqrt(np.mean(np.sum((predicted.reshape(-1,2)-pt.reshape(-1,2))**2,axis=1))))})
    data={'model':'brown_k1_k2_p1_p2','synthetic':a.synthetic,'width':size[0],'height':size[1],
          'camera_matrix':K.tolist(),'distortion':D.reshape(-1)[:4].tolist(),'fixed_k3':0,
          'rms_reprojection_pixels':float(rms),'per_view':errors,'checkerboard_inner_corners':[a.cols,a.rows],'square_mm':a.square_mm,
          'validation':'Training reprojection error only. Validate on separate views and inspect edges; not a guarantee of wide-angle model suitability.'}
    if a.model=='fisheye':
        focal_out=size[0]/(2*math.tan(math.radians(122/2)))
        data.update(model='opencv_fisheye_k1_k2_k3_k4',identity_mode=False,
            output_camera_matrix=[[focal_out,0,(size[0]-1)/2],[0,focal_out,(size[1]-1)/2],[0,0,1]],
            roi_cropped=False,rectification_rotation=[[1,0,0],[0,1,0],[0,0,1]],output_horizontal_fov_deg=122)
        data.pop('fixed_k3')
    session_file=a.images/'capture_session.json'
    if session_file.is_file():
        session=json.loads(session_file.read_text(encoding='utf-8-sig'))
        if (session.get('width'),session.get('height'))!=size:
            raise ValueError('Capture session resolution does not match the photographs')
        data['sensor_mode']=session.get('sensor_mode')
        data['capture_session']=session
    (a.out/'camera.json').write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')
    print(f'Accepted {len(objects)} views, training RMS={rms:.4f} pixels. Wrote {a.out / "camera.json"}')

if __name__=='__main__': main()
