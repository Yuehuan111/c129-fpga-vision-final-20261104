"""Deterministic vector test targets: A4 landscape, exact nominal mm geometry."""
import json
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

ROOT=Path(__file__).resolve().parents[1]
pdfmetrics.registerFont(TTFont('ChartChinese','C:/Windows/Fonts/msyh.ttc',subfontIndex=0))


def text(c,x,y,t,size=10):
    c.setFillGray(0);c.setFont('ChartChinese',size);c.drawString(x*mm,y*mm,t)


def scale(c):
    c.setStrokeGray(0);c.setLineWidth(.2*mm)
    c.line(18*mm,20*mm,118*mm,20*mm)
    for i in range(11):
        x=(18+i*10)*mm;c.line(x,18*mm,x,22*mm)
    text(c,18,12,'横向 100 mm：打印后用尺复核',9)
    c.line(281*mm,50*mm,281*mm,150*mm)
    for i in range(11):
        y=(50+i*10)*mm;c.line(279*mm,y,283*mm,y)
    c.saveState();c.translate(289*mm,50*mm);c.rotate(90)
    c.setFont('ChartChinese',9);c.drawString(0,0,'纵向 100 mm');c.restoreState()


def page_header(c,title,subtitle):
    text(c,14,196,title,16);text(c,14,186,subtitle,10)
    text(c,132,12,'A4 横向 | 实际大小 100% | 禁止适合页面 / 自动缩放',8)


def main():
    folder=ROOT/'charts';folder.mkdir(exist_ok=True)
    c=canvas.Canvas(str(folder/'A4_calibration_and_quality_targets.pdf'),pagesize=(297*mm,210*mm),pageCompression=1)
    c.setTitle('FPGA camera calibration and quality targets - A4 100 percent')
    c.setAuthor('Offline optical test kit')
    page_header(c,'01  标定棋盘 / 姿态与格距测试','11 × 6 个方格；OpenCV 内角点 cols=10, rows=5；方格边长 20 mm')
    x0,y0,s=38.5,45,20
    for row in range(6):
        for col in range(11):
            if (row+col)%2==0:
                c.setFillGray(0);c.rect((x0+col*s)*mm,(y0+row*s)*mm,s*mm,s*mm,fill=1,stroke=0)
    text(c,38.5,35,'有效棋盘 220 × 120 mm；保留白边，贴于平整硬板；纸张打印不等于高精度标定板。',9)
    scale(c);c.showPage()
    page_header(c,'02  直线度 / 边缘形变测试网格','间距 10 mm；线宽 0.25 mm；有效网格 240 × 140 mm；正对及斜拍均可测直线度')
    x0,y0=20,33
    c.setStrokeGray(0);c.setLineWidth(.25*mm)
    for i in range(25):c.line((x0+10*i)*mm,y0*mm,(x0+10*i)*mm,(y0+140)*mm)
    for i in range(15):c.line(x0*mm,(y0+10*i)*mm,(x0+240)*mm,(y0+10*i)*mm)
    # Crosshair outside the active grid, not extra false lines inside it.
    text(c,130,22,'分别记录中央、四边、四角的线直度误差；格距均匀性需先约束拍摄姿态。',8)
    scale(c);c.showPage()
    page_header(c,'03  清晰度 / 斜边 SFR 与 MTF50 测试','5 个斜边方块，边长 36 mm、旋转 5°；检测中心及四角；每块可选近竖直边和近水平边')
    for tag,x,y in [('TL',53,150),('TR',244,150),('C',148.5,103),('BL',53,62),('BR',244,62)]:
        c.saveState();c.translate(x*mm,y*mm);c.rotate(5)
        c.setFillGray(.5);c.rect(-18*mm,-18*mm,36*mm,36*mm,fill=1,stroke=0);c.restoreState()
        text(c,x-5,y-26,tag,10)
    text(c,14,28,'名义灰度 50%；实际反射对比度受打印影响，未标定。用于工程对比，不是认证 ISO 12233 图卡。',8)
    scale(c);c.showPage();c.save()
    manifest={'page_size_mm':[297,210],'print_scale_percent':100,'pages':[
        {'page':1,'type':'checkerboard','squares':[11,6],'inner_corners':[10,5],'square_mm':20,'active_size_mm':[220,120]},
        {'page':2,'type':'straight_grid','spacing_mm':10,'line_width_mm':.25,'active_size_mm':[240,140]},
        {'page':3,'type':'slanted_edge','square_mm':36,'angle_degrees':5,'patch_count':5,'nominal_device_gray':.5,'reflectance_contrast':'not calibrated','iso_certified':False}],
        'print_note':'Flat matte mounting, no Fit/Shrink. Measure both 100mm rulers and actual square spacing. These are vector artwork, not precision-certified physical targets.'}
    (folder/'target_specifications.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(folder/'A4_calibration_and_quality_targets.pdf')

if __name__=='__main__':main()
