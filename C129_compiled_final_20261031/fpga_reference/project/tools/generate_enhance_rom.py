from pathlib import Path
import math
ROOT=Path(__file__).resolve().parents[1]
GAMMAS=(.65,.75,.85,1.,1.15,1.30,1.45)
def gain_value(y, gamma, contrast):
    if not y: return 256
    x=y/255.0
    u=x**gamma
    if contrast: u += .5*u*(1-u)*(2*u-1)
    gain=min(448.,max(128.,256.*u/x))
    if gain>256 and y<16: gain=256+(gain-256)*max(0.,(y-4)/12.)
    return int(math.floor(gain+.5))
def curve_values(gamma, contrast):
    values=[gain_value(y,gamma,contrast) for y in range(256)]
    # RGB565 gray steps sometimes raise only G. Correct the resulting one-code
    # R/B reversal by increasing at most two Q8 gain LSBs at that luma address.
    # This is output quantization correction, not image-dependent processing.
    previous=(0,0,0)
    for gray in range(256):
        r=gray>>3; g=gray>>2
        r8=(r<<3)|(r>>2); g8=(g<<2)|(g>>4)
        y=(r8+2*g8+r8)>>2
        cap=min(448,511-max(r8,g8))
        required=max([math.ceil((old*256-128)/code)
                      for code,old in zip((r,g,r),previous) if code]+[0])
        if required>cap: raise ValueError('Gray monotonicity exceeds headroom')
        values[y]=max(values[y],required)
        previous=tuple((code*min(values[y],cap)+128)>>8 for code in (r,g,r))
    return values

def main():
    rows=[]
    for contrast in range(2):
        for index in range(8):
            values=[256]*256 if index==7 else curve_values(GAMMAS[index],contrast)
            for y in range(256):
                gain=values[y]
                address=(contrast<<11)|(index<<8)|y
                rows.append(f"        tone_rom[12'd{address}] = 9'd{gain};")
    path=ROOT/'rtl'/'complete_image_enhance.v'
    text=path.read_text(encoding='utf-8-sig')
    start='    // BEGIN GENERATED GAMMA ROM\n'
    end='    // END GENERATED GAMMA ROM\n'
    before,rest=text.split(start,1)
    _,after=rest.split(end,1)
    path.write_text(before+start+'    initial begin\n'+'\n'.join(rows)+'\n    end\n'+end+after,encoding='utf-8')
    print('Updated static mathematical gamma ROM:',path)
if __name__=='__main__': main()
