#!/usr/bin/env python3
"""动态蓝块抓取：二维视觉伺服到夹爪标定像素，再执行相对下降。

一次性标定：
    把蓝块放到夹爪真正能夹住的位置，运行 calibrate，记录此时 pre 位的像素。

演示：
    蓝块可以放在 pre 相机视野内的不同位置；
    grab 会自动搜索、用 J1/J3 闭环对准，再下降夹取。
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg


CALIB_PATH = os.path.expanduser('~/visual_pick_calib.json')


def require_poses():
    data = cg.load_poses()
    need = ('pre', 'grasp', 'lift')
    missing = [name for name in need
               if not isinstance(data.get(name), list)
               or len(data[name]) != 6]
    if missing:
        raise SystemExit('缺少位姿：' + ', '.join(missing)
                         + '。需要先用 arm_teach.py 保存 pre/grasp/lift')
    return data


def move_pose(bus, pose, time_ms):
    target = cg.write_pose(bus, pose, time_ms)
    time.sleep(max(0.6, time_ms / 1000.0 + 0.3))
    return target


def move_delta(bus, base, j1_delta, radial_delta, radial_index, time_ms):
    raw = list(base)
    raw[0] += int(round(j1_delta))
    raw[radial_index] += int(round(radial_delta))
    return move_pose(bus, raw, time_ms)


def set_grip(bus, value, time_ms=600):
    raw = cg.read_positions(bus)
    raw[5] = value
    cg.write_pose(bus, raw, time_ms)
    time.sleep(time_ms / 1000.0 + 0.4)


def detect(source, color, min_area, tries=4):
    for _ in range(tries):
        frame = source.read()
        if frame is not None:
            found = cg.detect(frame, color, min_area)
            if found is not None:
                return frame, found
        time.sleep(0.1)
    return None, None


def require_detect(source, color, min_area, label):
    frame, found = detect(source, color, min_area, tries=6)
    if found is None:
        raise SystemExit(f'{label}：没有识别到 {color}，检查 8082 画面和颜色阈值')
    return frame, found


def require_camera(source):
    if source.read() is None:
        raise SystemExit(
            f'取不到机械臂相机画面：{source.source}。'
            '先确认 8082 的 /snapshot 返回 200，再重跑')


def scan_workspace(source, bus, args, radial_index):
    frame, found = detect(source, args.color, args.min_area, tries=3)
    if found is not None:
        return found, cg.read_positions(bus)

    base = cg.read_positions(bus)
    print('当前画面没有色块，开始扫描工作区')
    for pitch in (0, -args.scan_pitch, args.scan_pitch):
        row = list(base)
        row[radial_index] += pitch
        if pitch:
            move_pose(bus, row, args.scan_time)
        for direction in (1, -1):
            raw = list(row)
            for index in range(args.scan_count):
                raw = move_delta(
                    bus, raw, direction * args.scan_step, 0,
                    radial_index, args.scan_time)
                frame, found = detect(
                    source, args.color, args.min_area, tries=2)
                print(f'  scan pitch={pitch} step={direction * (index + 1)} '
                      f'J1={raw[0]} -> {"找到" if found else "没有"}')
                if found is not None:
                    return found, raw
        if pitch:
            move_pose(bus, base, args.scan_time)
    return None, base


def probe_joint(bus, source, base, base_pixel, color, min_area,
                joint, step, settle):
    label = f'J{joint + 1}'
    for signed_step in (step, -step):
        target = list(base)
        target[joint] += signed_step
        actual = move_pose(bus, target, 450)
        frame, found = detect(source, color, min_area, tries=3)
        move_pose(bus, base, 450)
        travel = float(actual[joint] - base[joint])
        if abs(travel) < 1.0:
            continue
        if found is None:
            print(f'  探测 {label} step={signed_step}：目标丢失')
            continue
        name, area, cx, cy, _ = found
        dx = (cx - base_pixel[0]) / travel
        dy = (cy - base_pixel[1]) / travel
        print(f'  {label}: du={dx:.3f} dv={dy:.3f} px/raw')
        time.sleep(settle)
        return np.array([dx, dy], dtype=float)
    return None


def measure_jacobian(bus, source, args, radial_index, base, base_pixel):
    columns = {}
    for joint in (0, radial_index):
        column = probe_joint(
            bus, source, base, base_pixel, args.color, args.min_area,
            joint, args.probe_step, args.settle)
        if column is None:
            raise SystemExit(f'J{joint + 1} 探测失败，检查目标是否在画面边缘')
        columns[joint] = column

    matrix = np.column_stack((columns[0], columns[radial_index]))
    determinant = float(np.linalg.det(matrix))
    print(f'  局部雅可比：{matrix.tolist()} det={determinant:.3f}')
    if abs(determinant) < 0.08:
        raise SystemExit('局部雅可比太弱，换 --radial-joint 2/3 或减小 --probe-step')
    return matrix


def load_calib():
    if not os.path.exists(CALIB_PATH):
        raise SystemExit(f'找不到 {CALIB_PATH}，先运行 calibrate')
    with open(CALIB_PATH, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def save_calib(data):
    tmp = CALIB_PATH + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write('\n')
    os.replace(tmp, CALIB_PATH)


def cmd_calibrate(args):
    poses = require_poses()
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()

    require_camera(source)
    print('把蓝块放到夹爪真正能夹住的位置，再让机械臂停在 pre')
    move_pose(bus, poses['pre'], 1200)
    set_grip(bus, args.grip_open)

    frame, found = require_detect(
        source, args.color, args.min_area, '标定 pre 位')
    name, area, cx, cy, box = found
    print(f'标定目标：{name} area={int(area)} cx={cx} cy={cy} box={box}')
    if cx < 20 or cx > args.width - 20 or cy < 20 or cy > args.height - 20:
        print('警告：目标太靠近画面边缘，建议把镜头稍微抬高再标定')

    data = {
        'color': args.color,
        'target_cx': float(cx),
        'target_cy': float(cy),
        'target_area': float(area),
        'pre': poses['pre'],
        'grasp_delta': [poses['grasp'][i] - poses['pre'][i] for i in range(6)],
        'lift_delta': [poses['lift'][i] - poses['pre'][i] for i in range(6)],
    }
    save_calib(data)
    if args.save:
        import cv2
        cv2.imwrite(args.save, cg.annotate(frame.copy(), found))
        print(f'标注图：{args.save}')
    print(f'标定完成：target=({cx},{cy})')
    print(f'文件：{CALIB_PATH}')


def cmd_grab(args):
    calib = load_calib()
    poses = require_poses()
    if calib.get('color') != args.color:
        raise SystemExit(
            f"标定颜色是 {calib.get('color')}，现在却是 {args.color}；先重新 calibrate")

    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()
    radial_index = args.radial_joint - 1
    target = np.array(
        [float(calib['target_cx']), float(calib['target_cy'])], dtype=float)
    if args.target_cx is not None:
        target[0] = args.target_cx
    if args.target_cy is not None:
        target[1] = args.target_cy

    require_camera(source)
    print(f'到 pre，搜索 {args.color}')
    move_pose(bus, poses['pre'], 1200)
    set_grip(bus, args.grip_open)
    found, raw = scan_workspace(source, bus, args, radial_index)
    if found is None:
        raise SystemExit('扫描完仍没找到色块，检查是否在机械臂可达视野内')

    name, area, cx, cy, _ = found
    base_pixel = (cx, cy)
    print(f'找到：{name} area={int(area)} cx={cx} cy={cy}，目标像素 {target.tolist()}')
    raw = cg.read_positions(bus)
    matrix = measure_jacobian(
        bus, source, args, radial_index, raw, base_pixel)

    aligned = raw
    for index in range(args.max_tries):
        frame, found = require_detect(
            source, args.color, args.min_area, f'对准 {index + 1}')
        name, area, cx, cy, _ = found
        error = target - np.array([cx, cy], dtype=float)
        print(f'  try={index + 1} cx={cx} cy={cy} '
              f'err=({error[0]:.0f},{error[1]:.0f})')
        if abs(error[0]) <= args.tolerance and abs(error[1]) <= args.tolerance:
            print('  已对准夹爪标定像素')
            aligned = cg.read_positions(bus)
            break

        delta = np.linalg.solve(matrix, error)
        delta[0] = max(-args.max_j1_step, min(args.max_j1_step, delta[0]))
        delta[1] = max(-args.max_radial_step, min(args.max_radial_step, delta[1]))
        print(f'    ΔJ1={delta[0]:+.0f} ΔJ{radial_index + 1}={delta[1]:+.0f}')
        aligned = move_delta(
            bus, aligned, delta[0], delta[1], radial_index, args.move_time)
    else:
        raise SystemExit('视觉闭环未收敛：先减小 --probe-step 或 --max-step')

    if args.dry_run:
        print('干跑：已经视觉对准，不下降')
        return

    grasp_delta = np.array(calib['grasp_delta'], dtype=float)
    grasp = np.array(aligned, dtype=float) + grasp_delta
    grasp[5] = args.grip_open
    print(f'下降：{cg.fmt_pose(grasp.astype(int).tolist())}')
    move_pose(bus, grasp.astype(int).tolist(), args.grasp_time)
    set_grip(bus, args.grip_close, 700)

    lift_delta = np.array(calib['lift_delta'], dtype=float)
    lift = np.array(aligned, dtype=float) + lift_delta
    lift[5] = args.grip_close
    print(f'抬起：{cg.fmt_pose(lift.astype(int).tolist())}')
    move_pose(bus, lift.astype(int).tolist(), args.lift_time)
    print(f'抓取完成：{name} area={int(area)}')


def build_parser():
    parser = argparse.ArgumentParser(description='动态蓝块视觉抓取')
    sub = parser.add_subparsers(dest='command', required=True)

    common = {
        'color': dict(default='blue',
                      choices=('red', 'green', 'blue', 'yellow')),
        'min_area': dict(type=float, default=120.0),
        'camera': dict(default='http://127.0.0.1:8082/snapshot'),
        'grip_open': dict(type=int, default=1500),
    }

    calibrate = sub.add_parser('calibrate', help='记录夹爪标定像素')
    calibrate.add_argument('--color', **common['color'])
    calibrate.add_argument('--min-area', **common['min_area'])
    calibrate.add_argument('--camera', **common['camera'])
    calibrate.add_argument('--grip-open', **common['grip_open'])
    calibrate.add_argument('--save', default='')
    calibrate.add_argument('--width', type=int, default=640)
    calibrate.add_argument('--height', type=int, default=480)
    calibrate.set_defaults(func=cmd_calibrate)

    grab = sub.add_parser('grab', help='搜索、伺服对准并抓取')
    grab.add_argument('--color', **common['color'])
    grab.add_argument('--min-area', **common['min_area'])
    grab.add_argument('--camera', **common['camera'])
    grab.add_argument('--grip-open', **common['grip_open'])
    grab.add_argument('--grip-close', type=int, default=2550)
    grab.add_argument('--target-cx', type=float, default=None)
    grab.add_argument('--target-cy', type=float, default=None)
    grab.add_argument('--radial-joint', type=int, choices=(2, 3), default=3)
    grab.add_argument('--probe-step', type=int, default=25)
    grab.add_argument('--tolerance', type=float, default=14.0)
    grab.add_argument('--max-tries', type=int, default=8)
    grab.add_argument('--max-j1-step', type=float, default=55.0)
    grab.add_argument('--max-radial-step', type=float, default=45.0)
    grab.add_argument('--move-time', type=int, default=500)
    grab.add_argument('--grasp-time', type=int, default=900)
    grab.add_argument('--lift-time', type=int, default=1200)
    grab.add_argument('--settle', type=float, default=0.4)
    grab.add_argument('--scan-step', type=int, default=80)
    grab.add_argument('--scan-count', type=int, default=6)
    grab.add_argument('--scan-pitch', type=int, default=70)
    grab.add_argument('--scan-time', type=int, default=450)
    grab.add_argument('--dry-run', action='store_true')
    grab.set_defaults(func=cmd_grab)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
