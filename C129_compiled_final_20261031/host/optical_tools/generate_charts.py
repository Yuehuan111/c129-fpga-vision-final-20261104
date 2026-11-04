"""Generate engineering SVG/PNG targets; no PDF authoring or device access."""
from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


class Page:
    def __init__(self, title, dpi=300):
        self.dpi = dpi
        self.scale = dpi / 25.4
        self.image = Image.new('RGB', (round(297*self.scale), round(210*self.scale)), 'white')
        self.draw = ImageDraw.Draw(self.image)
        self.svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210">',
                    '<rect width="297" height="210" fill="white"/>']
        self.text(14, 13, title, 5)
        self.text(126, 204, 'A4 landscape | Actual size 100% | NO Fit / Shrink', 2.7)
        self.line(18, 191, 118, 191, .2)
        for i in range(11): self.line(18+10*i, 189, 18+10*i, 193, .2)
        self.text(18, 199, '100 mm: verify with a ruler after printing', 2.7)
        self.line(281, 40, 281, 140, .2)
        for i in range(11): self.line(279, 40+10*i, 283, 40+10*i, .2)
        self.text(267, 150, '100 mm', 2.7)

    def pt(self, x, y): return round(x*self.scale), round(y*self.scale)

    def text(self, x, y, value, mm_size=3):
        try: font = ImageFont.truetype('arial.ttf', max(8, round(mm_size*self.scale)))
        except OSError: font = ImageFont.load_default()
        self.draw.text(self.pt(x, y-mm_size), value, font=font, fill='black')
        self.svg.append(f'<text x="{x}" y="{y}" font-family="Arial,sans-serif" font-size="{mm_size}">{html.escape(value)}</text>')

    def line(self, x1, y1, x2, y2, width=.25):
        self.draw.line([self.pt(x1,y1), self.pt(x2,y2)], fill='black', width=max(1,round(width*self.scale)))
        self.svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="black" stroke-width="{width}"/>')

    def polygon(self, points, gray=0):
        value = round(gray*255)
        self.draw.polygon([self.pt(x,y) for x,y in points], fill=(value,value,value))
        self.svg.append('<polygon points="'+' '.join(f'{x:.8f},{y:.8f}' for x,y in points)+f'" fill="rgb({value},{value},{value})"/>')

    def save(self, folder, stem, formats):
        files=[]
        if 'svg' in formats:
            p=folder/(stem+'.svg');p.write_text('\n'.join(self.svg+['</svg>']),encoding='utf-8');files.append(p.name)
        if 'png' in formats:
            p=folder/(stem+'.png');self.image.save(p,dpi=(self.dpi,self.dpi));files.append(p.name)
        return files


def generate(out, cols=10, rows=5, square_mm=20., dpi=300, format='both'):
    if isinstance(cols,bool) or isinstance(rows,bool) or not isinstance(cols,int) or not isinstance(rows,int) or min(cols,rows)<3:
        raise ValueError('Inner corner dimensions must be integers >=3')
    if not math.isfinite(square_mm) or square_mm<=0 or (cols+1)*square_mm>240 or (rows+1)*square_mm>140:
        raise ValueError('Checkerboard must fit the 240 x 140 mm printable area')
    if dpi not in (150,300,600) or format not in ('svg','png','both'):
        raise ValueError('Unsupported format or DPI')
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    formats=('svg','png') if format=='both' else (format,)
    page=Page('01 Calibration checkerboard',dpi)
    page.text(14,23,f'{cols+1} x {rows+1} squares | {cols} x {rows} INNER corners | square {square_mm:g} mm',3.2)
    width,height=(cols+1)*square_mm,(rows+1)*square_mm
    x0,y0=(297-width)/2,35+(140-height)/2
    for row in range(rows+1):
        for col in range(cols+1):
            if (row+col)%2==0:
                x,y=x0+col*square_mm,y0+row*square_mm
                page.polygon([(x,y),(x+square_mm,y),(x+square_mm,y+square_mm),(x,y+square_mm)])
    files=page.save(out,'01_checkerboard',formats)
    page=Page('02 Line straightness grid',dpi)
    page.text(14,23,'10 mm spacing | 0.25 mm line width | active area 240 x 140 mm',3.2)
    for i in range(25):page.line(20+10*i,35,20+10*i,175)
    for i in range(15):page.line(20,35+10*i,260,35+10*i)
    files+=page.save(out,'02_straight_grid',formats)
    page=Page('03 Slanted edges for approximate SFR',dpi)
    page.text(14,23,'36 mm squares | 5 deg rotation | nominal gray 50% | NOT ISO certified',3.2)
    angle=math.radians(5)
    for label,cx,cy in [('TL',60,55),('TR',237,55),('C',148.5,105),('BL',60,155),('BR',237,155)]:
        points=[(cx+x*math.cos(angle)-y*math.sin(angle),cy+x*math.sin(angle)+y*math.cos(angle))
                for x,y in [(-18,-18),(18,-18),(18,18),(-18,18)]]
        page.polygon(points,.5);page.text(cx-4,cy+26,label,3.2)
    files+=page.save(out,'03_slanted_edges',formats)
    manifest={'page_mm':[297,210],'print_percent':100,'inner_corners':[cols,rows],
              'square_mm':square_mm,'dpi':dpi,'files':files,'precision_certified':False,'iso_certified':False,
              'note':'Engineering artwork. Print at actual size on matte flat rigid backing. Verify both rulers and actual square size.'}
    (out/'target_specifications.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    return manifest


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--cols',type=int,default=10)
    p.add_argument('--rows',type=int,default=5);p.add_argument('--square-mm',type=float,default=20)
    p.add_argument('--dpi',type=int,choices=[150,300,600],default=300)
    p.add_argument('--format',choices=['svg','png','both'],default='both')
    a=p.parse_args(argv);print(json.dumps(generate(a.out,a.cols,a.rows,a.square_mm,a.dpi,a.format),indent=2))


if __name__=='__main__':main()
