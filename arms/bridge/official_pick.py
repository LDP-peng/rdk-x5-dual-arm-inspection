#!/usr/bin/env python3
"""官方位姿动态抓取：top 观察 + blue 下降 + J1/J3 视觉偏移。

不需要示教 grab_poses.json。

一次性标定：
    先用官方 blue 位放蓝块，再升到 top，记录蓝块在 top 画面的像素。

演示：
    蓝块换位置后，在 top 画面搜索，J1/J3 闭环到标定像素；
    然后移动到官方 blue 位，并叠加同样的 J1/J3 偏移。
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg
import visual_pick as vp


CALIB_PATH = os.path.expanduser('~/official_pick_calib.json')


def preset_raw(name, current, grip=None):
    if name not in cg.PRESETS_DEG:
        raise SystemExit('官方位姿不存在：' + name)
    raw = cg.deg_pose(cg.PRESETS_DEG[name], current)
    if grip is not None:
        raw[5] = grip
    return raw


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
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()
    vp.require_camera(source)

    current = cg.read_positions(bus)
    grasp = preset_raw(args.grasp_pose, current, args.grip_open)
    print(f'移动到官方 {args.grasp_pose} 位：{cg.fmt_pose(grasp)}')
    vp.move_pose(bus, grasp, args.move_time)
    vp.set_grip(bus, args.grip_open)

    input('把蓝块放到夹爪真正能夹住的位置，手离开后按回车...')
    current = cg.read_positions(bus)
    scan = preset_raw(args.scan_pose, current, args.grip_open)
    print(f'移动到官方 {args.scan_pose} 位：{cg.fmt_pose(scan)}')
    vp.move_pose(bus, scan, args.move_time)
    vp.set_grip(bus, args.grip_open)

    frame, found = vp.require_detect(
        source, args.color, args.min_area, f'标定 {args.scan_pose} 位')
    name, area, cx, cy, box = found
    print(f'标定目标：{name} area={int(area)} cx={cx} cy={cy} box={box}')
    if cx < 20 or cx > 620 or cy < 20 or cy > 460:
        print('警告：目标太靠近边缘，换 look_at 观察位或调整相机安装')

    data = {
        'color': args.color,
        'scan_pose': args.scan_pose,
        'grasp_pose': args.grasp_pose,
        'target_cx': float(cx),
        'target_cy': float(cy),
        'target_area': float(area),
    }
    save_calib(data)
    if args.save:
        import cv2
        cv2.imwrite(args.save, cg.annotate(frame.copy(), found))
    print(f'标定完成：target=({cx},{cy})')
    print(f'文件：{CALIB_PATH}')


def cmd_grab(args):
    calib = load_calib()
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()
    vp.require_camera(source)

    if calib.get('color') != args.color:
        raise SystemExit(
            f"标定颜色是 {calib.get('color')}，现在却是 {args.color}；先重新 calibrate")

    scan_name = args.scan_pose or calib.get('scan_pose', 'top')
    grasp_name = args.grasp_pose or calib.get('grasp_pose', 'blue')
    target = np.array(
        [float(calib['target_cx']), float(calib['target_cy'])], dtype=float)
    if args.target_cx is not None:
        target[0] = args.target_cx
    if args.target_cy is not None:
        target[1] = args.target_cy

    current = cg.read_positions(bus)
    scan = preset_raw(scan_name, current, args.grip_open)
    print(f'到官方 {scan_name} 观察位：{cg.fmt_pose(scan)}')
    vp.move_pose(bus, scan, args.move_time)
    vp.set_grip(bus, args.grip_open)

    radial_index = args.radial_joint - 1
    found, raw = vp.scan_workspace(source, bus, args, radial_index)
    if found is None:
        raise SystemExit('扫描完仍没找到色块，检查是否在观察位视野内')
    name, area, cx, cy, _ = found
    base_pixel = (cx, cy)
    print(f'找到：{name} area={int(area)} cx={cx} cy={cy}，目标像素 {target.tolist()}')

    raw = cg.read_positions(bus)
    matrix = vp.measure_jacobian(
        bus, source, args, radial_index, raw, base_pixel)
    aligned = raw
    for index in range(args.max_tries):
        frame, found = vp.require_detect(
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
        aligned = vp.move_delta(
            bus, aligned, delta[0], delta[1], radial_index, args.move_time)
    else:
        raise SystemExit('视觉闭环未收敛：减小 --max-step 或重做标定')

    j1_offset = aligned[0] - scan[0]
    radial_offset = aligned[radial_index] - scan[radial_index]
    print(f'视觉偏移：J1={j1_offset:+d} J{radial_index + 1}={radial_offset:+d}')

    if args.dry_run:
        print('干跑：已对准，不执行下降')
        return

    current = cg.read_positions(bus)
    grasp = preset_raw(grasp_name, current, args.grip_open)
    grasp[0] += j1_offset
    grasp[radial_index] += radial_offset
    print(f'下降到官方 {grasp_name} 位 + 偏移：{cg.fmt_pose(grasp)}')
    vp.move_pose(bus, grasp, args.grasp_time)
    vp.set_grip(bus, args.grip_close, 700)

    current = cg.read_positions(bus)
    lift = preset_raw(scan_name, current, args.grip_close)
    lift[0] += j1_offset
    lift[radial_index] += radial_offset
    print(f'抬起：{cg.fmt_pose(lift)}')
    vp.move_pose(bus, lift, args.lift_time)
    print(f'抓取完成：{name} area={int(area)}')


def build_parser():
    parser = argparse.ArgumentParser(description='官方位姿动态抓取')
    sub = parser.add_subparsers(dest='command', required=True)

    common = {
        'color': dict(default='blue',
                      choices=('red', 'green', 'blue', 'yellow')),
        'min_area': dict(type=float, default=120.0),
        'camera': dict(default='http://127.0.0.1:8082/snapshot'),
        'grip_open': dict(type=int, default=1500),
    }

    calibrate = sub.add_parser('calibrate', help='记录官方观察位的夹爪像素')
    calibrate.add_argument('--color', **common['color'])
    calibrate.add_argument('--min-area', **common['min_area'])
    calibrate.add_argument('--camera', **common['camera'])
    calibrate.add_argument('--grip-open', **common['grip_open'])
    calibrate.add_argument('--scan-pose', default='top',
                           choices=tuple(cg.PRESETS_DEG))
    calibrate.add_argument('--grasp-pose', default='blue',
                           choices=tuple(cg.PRESETS_DEG))
    calibrate.add_argument('--move-time', type=int, default=1500)
    calibrate.add_argument('--save', default='')
    calibrate.set_defaults(func=cmd_calibrate)

    grab = sub.add_parser('grab', help='搜索、对准并抓取')
    grab.add_argument('--color', **common['color'])
    grab.add_argument('--min-area', **common['min_area'])
    grab.add_argument('--camera', **common['camera'])
    grab.add_argument('--grip-open', **common['grip_open'])
    grab.add_argument('--grip-close', type=int, default=2550)
    grab.add_argument('--scan-pose', default=None,
                      choices=tuple(cg.PRESETS_DEG))
    grab.add_argument('--grasp-pose', default=None,
                      choices=tuple(cg.PRESETS_DEG))
    grab.add_argument('--target-cx', type=float, default=None)
    grab.add_argument('--target-cy', type=float, default=None)
    grab.add_argument('--radial-joint', type=int, choices=(2, 3), default=3)
    grab.add_argument('--probe-step', type=int, default=25)
    grab.add_argument('--tolerance', type=float, default=14.0)
    grab.add_argument('--max-tries', type=int, default=8)
    grab.add_argument('--max-j1-step', type=float, default=55.0)
    grab.add_argument('--max-radial-step', type=float, default=45.0)
    grab.add_argument('--move-time', type=int, default=700)
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
