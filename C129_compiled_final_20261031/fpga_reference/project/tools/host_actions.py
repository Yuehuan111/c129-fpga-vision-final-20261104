"""Host-side configuration helpers. Pixel correction remains in FPGA RTL."""
from pathlib import Path
from numbers import Integral
from functools import lru_cache
import json
import time
import numpy as np
from camera_config import configuration, packet, normalize_calibration
from reference import float_map, ORDER
from fixed_map_fast import vector_fixed_map as fixed_map
from make_boot_config import render
from tile_budget import tile_budget
from fisheye_config import IDENTITY as FISHEYE_IDENTITY
from fisheye_reference import MODEL as FISHEYE_MODEL
FISHEYE_BUILD_ID = 0x0720A001  # Historical 18-word sender must refuse complete C021.

ROOT = Path(__file__).resolve().parents[1]
IDENTITY = {'model': 'brown_k1_k2_p1_p2', 'width': 1280, 'height': 720,
            'synthetic': True, 'fixed_k3': 0,
            'camera_matrix': [[1024, 0, 639.5], [0, 1024, 359.5], [0, 0, 1]],
            'distortion': [0, 0, 0, 0]}


def brightness_configuration(words, gain_q8=256, offset=0):
    """Keep the complete current geometry; change only the two pixel controls.

    The existing 18-word commit protocol lets the FPGA reuse its geometry
    cache. No coordinate table is generated for upload.
    """
    result = validate_board_configuration(words, {'width': 1280, 'height': 720})
    if len(result) != 18:
        raise ValueError('高级版亮度控制需要当前完整的18字鱼眼配置。')
    if (isinstance(gain_q8, bool) or not isinstance(gain_q8, Integral) or
            not 0 <= gain_q8 <= 2048 or isinstance(offset, bool) or
            not isinstance(offset, Integral) or not -255 <= offset <= 255):
        raise ValueError('gain Q8 必须为0..2048整数，offset必须为-255..255整数。')
    result[14], result[15] = int(gain_q8), int(offset)
    return result

def validate_board_configuration(words, metadata):
    """Validate the complete packet contract before any file or serial mutation."""
    if not isinstance(metadata, dict) or any(
            isinstance(metadata.get(key), bool) or not isinstance(metadata.get(key), int)
            for key in ('width', 'height')) or (metadata['width'], metadata['height']) != (1280, 720):
        raise ValueError('本版 FPGA 参数必须明确标记 width=1280、height=720；不接受1080p参数。')
    if not isinstance(words, (list, tuple)) or len(words) not in (16,18):
        raise ValueError('必须提供完整参数：旧Brown16字或新版fisheye18字，再提交COMMIT。')
    if any(isinstance(value, bool) or not isinstance(value, Integral) or
           not -(1 << 31) <= value < (1 << 31) for value in words):
        raise ValueError('每个参数字必须是有符号32位整数。')
    result=[int(value) for value in words]
    if len(result)==18:
        from fisheye_reference import validate_words
        validate_words(result)
    return result


@lru_cache(maxsize=4)
def check_board_geometry(words):
    """Check public send/boot paths too; CLI callers need not use the GUI fitter."""
    if min(words[0],words[1],words[6],words[7]) <= 0:
        raise ValueError('焦距和倒焦距必须为正数。')
    if not 0 <= words[14] <= 2048 or not -255 <= words[15] <= 255:
        raise ValueError('亮度增益范围为0到8，整数偏移范围为-255到255。')
    if len(words)==18:
        from fisheye_reference import vector_fixed_map as mapper
    else:mapper=fixed_map
    entries,errors=mapper(1280,720,words)
    if errors:
        raise ValueError(f'定点坐标在{errors}个像素溢出，参数未发送。')
    invalid=int(np.count_nonzero((entries>>38)==0))
    if invalid==1280*720:
        raise ValueError('整幅映射均越界，参数未发送。')
    budget=tile_budget(entries,1280,720)
    if budget['unsupported_tiles']:
        raise ValueError('映射超过FPGA块缓存范围，参数未发送；请调整视场或旋转。')
    return {'invalid_pixels':invalid,'overflow_pixels':errors,'tile_budget':budget}

def load_calibration(path):
    d = normalize_calibration(json.loads(Path(path).read_text(encoding='utf-8-sig')))
    configuration(d)
    if (d['width'], d['height']) != (1280, 720):
        raise ValueError('需要 1280x720 原图标定；不要拉伸或裁剪标定图片。')
    if d.get('synthetic') is not False:
        raise ValueError('上板参数必须来自真实镜头：synthetic 必须为 false。')
    if d.get('sensor_mode') not in (None,'ov5640_dvp_rgb565_wide_1280x720_30_candidate_v1'):
        raise ValueError('标定文件的摄像头模式与本工程不一致；相同分辨率也不能代替相同裁剪模式。')
    return d

def fitted_configuration(d, zoom=1.0, angle=0.0, auto_crop=True):
    """Suggest crop using float map, then certify coverage with the exact integer model.
    Coverage is not a certification of lens calibration accuracy.
    """
    d=normalize_calibration(d)
    w, h = d['width'], d['height']
    if d.get('model')==FISHEYE_MODEL:
        from fisheye_reference import vector_fixed_map as mapper,float_map as floating_mapper
    else:mapper,floating_mapper=fixed_map,float_map
    zoom = float(zoom)
    if not .25 <= zoom <= 8:
        raise ValueError('zoom 必须在 0.25 至 8 之间。')
    while True:
        words = configuration(d, angle=angle, zoom=zoom)
        if auto_crop:
            x, y = floating_mapper(w, h, words)
            approx_inside = bool(np.all((x >= .05) & (x <= w-1-.05) &
                                        (y >= .05) & (y <= h-1-.05)))
            if not approx_inside:
                zoom = round(zoom + .05, 6)
                if zoom > 8:
                    raise ValueError('无法取得有效裁剪区，请检查标定模型和主点。')
                continue
        entries, errors = mapper(w, h, words)
        invalid = int(np.count_nonzero((entries >> 38) == 0))
        if errors:
            raise ValueError(f'定点运算在 {errors} 个像素溢出，请重新检查参数。')
        if auto_crop and invalid:
            zoom = round(zoom + .05, 6)
            if zoom > 8:
                raise ValueError('定点映射仍有越界，请检查标定。')
            continue
        budget = tile_budget(entries, w, h)
        if budget['unsupported_tiles']:
            raise ValueError('当前角度/视场超过FPGA块缓存范围，请减小旋转或调整输出内参/zoom。'
                             +str(budget['unsupported_tiles'][:3]))
        return words, {'zoom': zoom, 'angle': angle, 'invalid_pixels': invalid,
                       'overflow_pixels': errors, 'width': w, 'height': h,
                       'tile_budget': budget}

def export_configuration(words, metadata, folder):
    words = validate_board_configuration(words, metadata)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    payload = b''.join(packet(i, v) for i, v in enumerate(words))
    payload += packet(0xff, 0x434f4d54)
    (folder/'uart_packets.bin').write_bytes(payload)
    (folder/'config.hex').write_text(''.join(f'{v & 0xffffffff:08x}\n' for v in words), encoding='ascii')
    if len(words)==18:
        from fisheye_reference import ORDER as export_order
    else:export_order=ORDER
    (folder/'config.json').write_text(json.dumps({**metadata,'register_count':len(words), 'words': dict(zip(export_order, words))},
                                               indent=2, ensure_ascii=False), encoding='utf-8')
    return payload

def send_configuration(port_name, words, metadata):
    # Called only by an explicit Send/Original GUI action, never at startup.
    words = validate_board_configuration(words, metadata)
    check_board_geometry(tuple(words))
    import serial
    from serial.tools import list_ports
    from read_board_status import extract_packets
    if not port_name:
        raise ValueError('请先选择板载 USB-UART 对应的 COM 口。')
    if not any(p.device == port_name and p.vid == 0x10c4 and p.pid == 0xea60
               for p in list_ports.comports()):
        raise ValueError('所选端口不是已确认的CP210x板载串口。')
    payload = export_configuration(words, metadata, ROOT/'output/last_config')
    expected_build=FISHEYE_BUILD_ID if len(words)==18 else 0x7200001
    with serial.Serial(port_name, 115200, timeout=.2, write_timeout=2) as port:
        buffer = bytearray()
        def await_status(predicate, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                buffer.extend(port.read(256))
                for status in extract_packets(buffer):
                    if status['build_id'] != expected_build:
                        raise ValueError('板端版本不匹配，不能发送本版参数。')
                    if predicate(status):
                        return status
            raise TimeoutError('未取得匹配的板端状态，不能认定参数已生效。')
        before = await_status(lambda s: not s['map_building'], 12)
        for start in range(0, len(payload), 8):
            n = port.write(payload[start:start+8])
            if n != 8:
                raise IOError('串口未发送完整数据包。')
            port.flush()
            time.sleep(.01)
        after = await_status(lambda s: s['commits'] != before['commits'] and
                             s['config_version'] != before['config_version'] and
                             not s['map_building'], 25)
    (ROOT/'output/last_config/status_confirmation.json').write_text(
        json.dumps({'before':before,'after':after}, indent=2), encoding='utf-8')
    return (f'映射表版本{after["config_version"]}已发布，尚未确认该版本结果已显示。'
            + ('板端仍有错误标志，请查看状态记录。' if after['error_sticky'] else ''))

def export_boot(words, metadata):
    words = validate_board_configuration(words, metadata)
    check_board_geometry(tuple(words))
    target = ROOT/'rtl/video_config_boot.v'
    backup = ROOT/'output/boot_backups'
    backup.mkdir(parents=True, exist_ok=True)
    if target.exists():
        stamp = time.time_ns()
        (backup/f'video_config_boot_{stamp}.v').write_bytes(target.read_bytes())
    target.write_text(render(words), encoding='ascii')
    import hashlib
    boot_metadata={**metadata,'words':words,'register_count':len(words),
                   'build_id':f'0x{FISHEYE_BUILD_ID:08x}' if len(words)==18 else '0x07200001',
                   'model':FISHEYE_MODEL if len(words)==18 else 'brown_k1_k2_p1_p2',
                   'rtl_sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
    (ROOT/'board/boot_config.json').write_text(json.dumps(boot_metadata, indent=2, ensure_ascii=False), encoding='utf-8')
    return '上电参数已写入 RTL；需重新运行 board/prepare_board.py --channel 1，再用 tools/build_project.ps1 -Channel 1 编译并下载。当前板上程序尚未改变。'
