"""Vectorized host-side validator. Hardware still builds and applies its own map."""
import numpy as np
from reference import Q

def vector_fixed_map(width, height, config, chunk_rows=64):
    """Same checked Q16.16 operation order as reference.fixed_map, vectorized."""
    result = np.zeros((height, width), dtype=np.uint64)
    total_errors = 0
    for first in range(0, height, chunk_rows):
        vv, uu = np.mgrid[first:min(first + chunk_rows, height), :width]
        invalid = np.zeros(uu.shape, dtype=bool)

        def check(value):
            nonlocal invalid
            value = np.asarray(value, dtype=np.int64)
            bad = (value < -(1 << 31)) | (value >= (1 << 31))
            invalid |= bad
            # Stop overflow propagation just as scalar reference aborts a pixel.
            return np.where(bad, 0, value)

        def add(a, b):
            return check(a + b)

        def mul(a, b):
            return check((np.asarray(a, dtype=np.int64) * b) // Q)

        c = config
        x = mul(check(uu * Q - c[4]), c[6]); y = mul(check(vv * Q - c[5]), c[7])
        rcx = mul(x, c[12]); rsy = mul(y, c[13]); rsx = mul(x, c[13]); rcy = mul(y, c[12])
        x = check(rcx - rsy); y = add(rsx, rcy)
        x2 = mul(x, x); y2 = mul(y, y); xy = mul(x, y); r2 = add(x2, y2); r4 = mul(r2, r2)
        radial = add(add(Q, mul(c[8], r2)), mul(c[9], r4))
        xr = mul(x, radial); yr = mul(y, radial); p1xy = mul(c[10], xy); p2xy = mul(c[11], xy)
        tx = add(r2, add(x2, x2)); ty = add(r2, add(y2, y2))
        p2tx = mul(c[11], tx); p1ty = mul(c[10], ty)
        xd = add(add(xr, add(p1xy, p1xy)), p2tx); yd = add(add(yr, p1ty), add(p2xy, p2xy))
        sx = add(mul(c[0], xd), c[2]); sy = add(mul(c[1], yd), c[3])
        xq = (sx + 128) >> 8; yq = (sy + 128) >> 8
        good = (~invalid & (sx >= 0) & (sx <= (width - 1) * Q) & (sy >= 0) & (sy <= (height - 1) * Q)
                & (xq <= (width - 1) * 256) & (yq <= (height - 1) * 256))
        result[first:first + len(uu)] = np.where(good, (1 << 38) | (xq << 19) | yq, 0).astype(np.uint64)
        total_errors += int(invalid.sum())
    return result, total_errors

