#!/usr/bin/env python3
"""俯视扫描抓取：色块像素坐标 -> J1/J2 闭环 -> 固定 pre/grasp/lift。

用法：
    1. 先标定像素到关节的增益：
       python3 ~/scan_grasp.py calibrate --color blue --block-cm 3.0
    2. 再扫描抓取：
       python3 ~/scan_grasp.py grab --color blue --grip-open 1500 \
           --offset-cm 5.5 --offset-dir down

offset-dir 是夹爪相对镜头在画面中的方向：left/right/up/down。
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg


CALIB_PATH = os.path.expanduser('~/scan_calib.json')
DIRS = {
    'left': (-1.0, 0.0),
    'right': (1.0, 0.0),
    'up': (0.0, -1.0),
    'down': (0.0, 1.0),
}


def load_poses():
    data = cg.load_poses()
    need = ('observe', 'look', 'pre', 'grasp', 'lift')
    missing = [name for name in need
               if not isinstance(data.get(name), list)
               or len(data[name]) != 6]
    if missing:
        raise SystemExit('缺少位姿：' + ', '.join(missing)
                         + '。先用 arm_teach.py 示教并 save。')
    return data


def detect(source, color, min_area, tries=6):
    for _ in range(tries):
        frame = source.read()
        if frame is not None:
            found = cg.detect(frame, color, min_area)
            if found is not None:
                return found
        time.sleep(0.12)
    return None


def require_detect(source, color, min_area, label):
    found = detect(source, color, min_area)
    if found is None:
        raise SystemExit(f'{label} 没有识别到 {color}，检查 8082 画面和积木位置')
    return found


def move_pose(bus, pose, time_ms):
    cg.write_pose(bus, pose, time_ms)
    time.sleep(max(0.5, time_ms / 1000.0 + 0.3))


def open_grip(bus, value):
    raw = cg.read_positions(bus)
    raw[5] = value
    cg.write_pose(bus, raw, 500)
    time.sleep(0.6)


def move_delta(bus, base, j1_delta, radial_delta, radial_index,
               time_ms=500):
    raw = list(base)
    raw[0] += int(round(j1_delta))
    raw[radial_index] += int(round(radial_delta))
    cg.write_pose(bus, raw, time_ms)
    time.sleep(max(0.5, time_ms / 1000.0 + 0.3))
    return raw


def with_offsets(pose, j1_offset, radial_offset, radial_index):
    result = list(pose)
    result[0] += int(round(j1_offset))
    result[radial_index] += int(round(radial_offset))
    return result


def sweep_find(source, bus, args, radial_index):
    base = cg.read_positions(bus)
    for direction in (1, -1):
        raw = base
        for index in range(args.sweep_count):
            raw = move_delta(bus, raw, direction * args.sweep_step, 0,
                             radial_index, args.sweep_time)
            found = detect(source, args.color, args.min_area, tries=3)
            print(f'  扫描 {index + 1}/{args.sweep_count} '
                  f'J1={raw[0]} -> '
                  f'{"找到" if found is not None else "没有"}')
            if found is not None:
                return found, raw
        move_delta(bus, base, 0, 0, radial_index, args.sweep_time)
    return None, base


def load_calib():
    if not os.path.exists(CALIB_PATH):
        raise SystemExit(f'找不到 {CALIB_PATH}，先运行 calibrate')
    with open(CALIB_PATH, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def save_calib(data):
    with open(CALIB_PATH, 'w', encoding='utf-8') as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
    print(f'标定文件：{CALIB_PATH}')


def target_pixel(calib, args):
    target_cx = float(calib.get('target_cx', 320.0))
    target_cy = float(calib.get('target_cy', 240.0))
    if args.target_cx is not None:
        target_cx = args.target_cx
    if args.target_cy is not None:
        target_cy = args.target_cy
    if args.offset_cm is not None:
        scale = args.px_per_cm or calib.get('px_per_cm')
        if not scale:
            raise SystemExit('需要 --px-per-cm，或先用 --block-cm 标定')
        dx, dy = DIRS[args.offset_dir]
        target_cx = 320.0 + args.offset_cm * scale * dx
        target_cy = 240.0 + args.offset_cm * scale * dy
    return target_cx, target_cy


def cmd_calibrate(args):
    poses = load_poses()
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()
    radial_index = args.radial_joint - 1

    move_pose(bus, poses['observe'], 1200)
    move_pose(bus, poses['look'], 1200)
    open_grip(bus, args.grip_open)

    found = require_detect(source, args.color, args.min_area, '标定起点')
    name, area, x0, y0, (bx, by, bw, bh) = found
    print(f'起点：{name} area={int(area)} cx={x0} cy={y0} box={bw}x{bh}')
    base = cg.read_positions(bus)

    move_delta(bus, base, args.step, 0, radial_index)
    found1 = require_detect(source, args.color, args.min_area, 'J1 标定')
    _, _, x1, y1, _ = found1
    du1 = (x1 - x0) / float(args.step)
    dv1 = (y1 - y0) / float(args.step)
    move_delta(bus, base, 0, 0, radial_index)
    print(f'J1：du={du1:.3f} dv={dv1:.3f} px/raw')

    move_delta(bus, base, 0, args.step, radial_index)
    found2 = require_detect(source, args.color, args.min_area, 'J2 标定')
    _, _, x2, y2, _ = found2
    du2 = (x2 - x0) / float(args.step)
    dv2 = (y2 - y0) / float(args.step)
    move_delta(bus, base, 0, 0, radial_index)
    print(f'J{args.radial_joint}：du={du2:.3f} dv={dv2:.3f} px/raw')

    matrix = np.array([[du1, du2], [dv1, dv2]], dtype=float)
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) < 1e-6:
        raise SystemExit('标定矩阵不可逆，换一个 --radial-joint 2 或 3，或加大 --step')
    if abs(dv1) + abs(dv2) < 0.35:
        print('警告：J1/J2 对画面纵向控制太弱，建议换 --radial-joint 3')

    scale = args.px_per_cm
    if args.block_cm and args.block_cm > 0:
        scale = ((bw + bh) / 2.0) / args.block_cm

    data = {
        'target_cx': 320.0,
        'target_cy': 240.0,
        'jacobian': matrix.tolist(),
        'radial_index': radial_index,
        'radial_joint': args.radial_joint,
        'px_per_cm': scale,
    }
    save_calib(data)
    print('标定矩阵：')
    print(matrix)
    if scale:
        print(f'像素比例：{scale:.2f} px/cm；5.5cm={5.5 * scale:.1f}px')
    print('下一步示例：')
    print('python3 ~/scan_grasp.py grab --color blue --grip-open 1500 '
          '--offset-cm 5.5 --offset-dir down')


def cmd_grab(args):
    calib = load_calib()
    poses = load_poses()
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()
    radial_index = int(calib.get('radial_index', 1))
    matrix = np.array(calib['jacobian'], dtype=float)
    if abs(float(np.linalg.det(matrix))) < 1e-6:
        raise SystemExit('标定矩阵不可逆，重新 calibrate')

    target_cx, target_cy = target_pixel(calib, args)
    print(f'目标像素：cx={target_cx:.1f} cy={target_cy:.1f}')

    move_pose(bus, poses['observe'], 1200)
    move_pose(bus, poses['look'], 1200)
    open_grip(bus, args.grip_open)

    found = detect(source, args.color, args.min_area)
    if found is None and not args.no_sweep:
        print('当前画面没有色块，开始 J1 扫描')
        found, raw = sweep_find(source, bus, args, radial_index)
    else:
        raw = cg.read_positions(bus)
    if found is None:
        raise SystemExit('扫描结束仍未找到色块：检查颜色阈值/积木是否在可达区')
    name, area, cx, cy, _ = found
    print(f'扫描：{name} area={int(area)} cx={cx} cy={cy}')

    for index in range(args.max_tries):
        found = require_detect(source, args.color, args.min_area, '闭环')
        name, area, cx, cy, _ = found
        error = np.array([target_cx - cx, target_cy - cy], dtype=float)
        print(f'  try={index + 1} cx={cx} cy={cy} '
              f'err=({error[0]:.0f},{error[1]:.0f})')
        if abs(error[0]) <= args.tolerance and abs(error[1]) <= args.tolerance:
            print('  已对准')
            break
        delta = np.linalg.solve(matrix, error)
        delta = np.clip(delta, -args.max_step, args.max_step)
        raw = move_delta(bus, raw, delta[0], delta[1], radial_index)
    else:
        raise SystemExit('闭环超时：减小 --offset-cm，或检查标定方向')

    raw = cg.read_positions(bus)
    j1_offset = raw[0] - poses['look'][0]
    radial_offset = raw[radial_index] - poses['look'][radial_index]
    print(f'偏移：J1={j1_offset} J{radial_index + 1}={radial_offset}')
    if args.dry_run:
        print('干跑：已对准，不执行 pre/grasp/lift')
        return

    pre = with_offsets(poses['pre'], j1_offset, radial_offset, radial_index)
    pre[5] = args.grip_open
    print('pre')
    move_pose(bus, pre, args.move_time)

    grasp = with_offsets(poses['grasp'], j1_offset, radial_offset, radial_index)
    grasp[5] = args.grip_open
    print('grasp')
    move_pose(bus, grasp, args.grasp_time)

    raw = cg.read_positions(bus)
    raw[5] = cg.GRIP_CLOSE
    cg.write_pose(bus, raw, 700)
    time.sleep(0.8)

    lift = with_offsets(poses['lift'], j1_offset, radial_offset, radial_index)
    lift[5] = cg.GRIP_CLOSE
    print('lift')
    move_pose(bus, lift, args.lift_time)
    print(f'抓取完成：{name} area={int(area)}')


def build_parser():
    parser = argparse.ArgumentParser(description='扫描坐标抓取')
    subparsers = parser.add_subparsers(dest='command', required=True)

    common = {
        'color': dict(default='blue', choices=('red', 'green', 'blue', 'yellow')),
        'min_area': dict(type=float, default=120.0),
        'camera': dict(default='http://127.0.0.1:8082/snapshot'),
        'grip_open': dict(type=int, default=1500),
    }

    calibrate = subparsers.add_parser('calibrate', help='标定像素到 J1/J2 的增益')
    calibrate.add_argument('--color', **common['color'])
    calibrate.add_argument('--min-area', **common['min_area'])
    calibrate.add_argument('--camera', **common['camera'])
    calibrate.add_argument('--grip-open', **common['grip_open'])
    calibrate.add_argument('--step', type=int, default=40,
                           help='标定时单关节转动 raw，目标跑出画面就减小')
    calibrate.add_argument('--radial-joint', type=int, choices=(2, 3), default=2,
                           help='用哪个关节做前后方向，默认 J2')
    calibrate.add_argument('--block-cm', type=float, default=None,
                           help='色块实际边长 cm，用于估算 px/cm')
    calibrate.add_argument('--px-per-cm', type=float, default=None,
                           help='已知像素比例时直接填写')
    calibrate.set_defaults(func=cmd_calibrate)

    grab = subparsers.add_parser('grab', help='扫描并抓取')
    grab.add_argument('--color', **common['color'])
    grab.add_argument('--min-area', **common['min_area'])
    grab.add_argument('--camera', **common['camera'])
    grab.add_argument('--grip-open', **common['grip_open'])
    grab.add_argument('--offset-cm', type=float, default=5.5,
                      help='镜头中心到夹爪中心的距离，默认5.5')
    grab.add_argument('--px-per-cm', type=float, default=None,
                      help='覆盖标定文件里的像素比例')
    grab.add_argument('--offset-dir', choices=tuple(DIRS), default='down',
                      help='夹爪在画面中相对镜头的方向')
    grab.add_argument('--target-cx', type=float, default=None)
    grab.add_argument('--target-cy', type=float, default=None)
    grab.add_argument('--tolerance', type=float, default=12.0)
    grab.add_argument('--max-tries', type=int, default=7)
    grab.add_argument('--max-step', type=int, default=70)
    grab.add_argument('--sweep-step', type=int, default=70,
                      help='扫描时每次转 J1 的 raw')
    grab.add_argument('--sweep-count', type=int, default=6,
                      help='左右各扫多少步')
    grab.add_argument('--sweep-time', type=int, default=450)
    grab.add_argument('--no-sweep', action='store_true',
                      help='不左右扫描，只看当前画面')
    grab.add_argument('--dry-run', action='store_true',
                      help='只做扫描和闭环，不下压抓取')
    grab.add_argument('--move-time', type=int, default=1200)
    grab.add_argument('--grasp-time', type=int, default=900)
    grab.add_argument('--lift-time', type=int, default=1200)
    grab.set_defaults(func=cmd_grab)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
