"""Check fixed-map tile envelopes against the implemented on-chip ROI capacity."""
import numpy as np

def tile_budget(entries, width, height, tile_width=128, tile_height=32,
                max_roi_width=256, max_roi_height=64):
    a = np.asarray(entries, dtype=np.uint64).reshape(height, width)
    max_w = max_h = source_bytes = 0
    unsupported = []
    for y in range(0, height, tile_height):
        for x in range(0, width, tile_width):
            t = a[y:y + tile_height, x:x + tile_width]
            valid = t[(t >> 38) != 0]
            if not valid.size:
                continue
            sx = ((valid >> 27) & 2047).astype(np.int64)
            sy = ((valid >> 8) & 2047).astype(np.int64)
            fx = ((valid >> 19) & 255) != 0
            fy = (valid & 255) != 0
            left = int(sx.min()) // 16 * 16
            # Match the RTL: zero fractional weight duplicates p00 and does
            # not require the next sample. Nonzero weights clamp at the edge.
            right = (int(np.minimum(width - 1, sx + fx).max()) + 16) // 16 * 16
            roi_w = right - left
            roi_h = int(np.minimum(height - 1, sy + fy).max()) - int(sy.min()) + 1
            max_w, max_h = max(max_w, roi_w), max(max_h, roi_h)
            source_bytes += roi_w * roi_h * 2
            if roi_w > max_roi_width or roi_h > max_roi_height:
                unsupported.append({'x': x, 'y': y, 'roi_width': roi_w, 'roi_height': roi_h})
    return {'tile_width':tile_width,'tile_height':tile_height,
            'cache_width':max_roi_width,'cache_height':max_roi_height,
            'max_roi_width': max_w, 'max_roi_height': max_h,
            'source_read_bytes_per_frame': source_bytes,
            'map_read_bytes_per_frame': width * height * 8,
            'unsupported_tiles': unsupported}
