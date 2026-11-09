#!/usr/bin/env python3
"""YOLO-triggered rear-arm grasp.

This keeps the proven pick_rear.py pose sequence and only swaps the visual
trigger from HSV to model0.pt.  The demo can use Vegetable_leaf as a trigger
for an apple core while still using the detected box centre for J1 alignment.

Before running:
    1. stop slave_joint_node.py;
    2. start the arm camera snapshot server on port 8082;
    3. make sure grab_poses.json contains observe/look/pre/grasp/lift.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg
import pick_rear as pr
import yolo_detect as yd


def get_target(source, detector, target_class, min_area, tries=4):
    for _ in range(tries):
        frame = source.read()
        if frame is not None:
            found = detector.find(frame, target_class, min_area)
            if found is not None:
                return found
        time.sleep(0.12)
    return None


def align_at(source, bus, detector, args, label, pose):
    print(f'[{label}] YOLO 识别触发类：{args.target_class}')
    found = get_target(source, detector, args.target_class, args.min_area)
    if found is None:
        if not args.fixed:
            raise SystemExit(
                f'{label} 没有识别到 {args.target_class}：'
                '检查 8082 图传、物体位置和 --conf')
        print('  未识别到目标，固定位姿继续抓取')
        return 0, args.target_class, 0.0

    name, area, cx, cy, _ = found
    print(f'  首次识别：{name} area={int(area)} cx={cx} cy={cy}')
    if args.calibrate:
        raw = cg.read_positions(bus)
        print(f'  当前 J1={raw[0]}；把 --target-cx 设为 {cx}')
        raise SystemExit(0)

    if args.fixed:
        print('  固定位姿：跳过 J1 对准')
        return 0, name, area

    print(f'  小步转 J1，对准目标像素 cx={args.target_cx:.0f}')
    for index in range(args.max_align_tries):
        found = get_target(source, detector, args.target_class, args.min_area)
        if found is None:
            raise SystemExit('对准过程中目标丢失，停止')
        name, area, cx, cy, _ = found
        error = cx - args.target_cx
        print(f'  try={index + 1} class={name} cx={cx} err={error:.0f}')
        if abs(error) <= args.center_tol:
            print('  已对准')
            break

        delta = int(0.5 * error)
        if delta == 0:
            delta = 40 if error > 0 else -40
        delta = max(-args.max_j1_step,
                    min(args.max_j1_step, delta)) * args.align_sign
        pr.jog_j1(bus, delta)
    else:
        raise SystemExit('J1 对准失败：可尝试 --align-sign -1')

    aligned = cg.read_positions(bus)
    j1_offset = aligned[0] - pose[0]
    print(f'  J1 对准偏移：{j1_offset}（后续位姿会跟随）')
    return j1_offset, name, area


def build_parser():
    parser = argparse.ArgumentParser(description='YOLO 触发 + 后置机械臂抓取')
    parser.add_argument('--weights', default='~/model0.pt')
    parser.add_argument('--repo', default='~/yolov5-6.2')
    parser.add_argument('--target-class', default='Vegetable_leaf',
                        help='触发类；支持逗号分隔，演示可用 Vegetable_leaf')
    parser.add_argument('--conf', type=float, default=0.2)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--torch-device', default='cpu')
    parser.add_argument('--min-area', type=float, default=2000.0)
    parser.add_argument('--camera', default='http://127.0.0.1:8082/snapshot')
    parser.add_argument('--move-time', type=int, default=1200)
    parser.add_argument('--grasp-time', type=int, default=900)
    parser.add_argument('--lift-time', type=int, default=1200)
    parser.add_argument('--grip-open', type=int, default=1500)
    parser.add_argument('--grip-close', type=int, default=2550)
    parser.add_argument('--close-time', type=int, default=900)
    parser.add_argument('--target-cx', type=float, default=285.0)
    parser.add_argument('--center-tol', type=float, default=35.0)
    parser.add_argument('--max-j1-step', type=int, default=100)
    parser.add_argument('--max-align-tries', type=int, default=8)
    parser.add_argument('--align-sign', type=int, choices=(1, -1), default=1)
    parser.add_argument('--align-pose', choices=('look', 'pre'), default='pre')
    parser.add_argument('--fixed', action='store_true')
    parser.add_argument('--calibrate', action='store_true')
    parser.add_argument('--dry-run', action='store_true',
                        help='只移动、检测和 J1 对准，不下压抓取')
    return parser


def main():
    args = build_parser().parse_args()
    poses = pr.require_poses()
    detector = yd.YoloDetector(
        args.weights, args.repo, args.torch_device, args.imgsz,
        args.conf, args.iou)
    source = cg.FrameSource(args.camera, '/dev/video0')
    bus = cg.open_bus()

    print('[1/6] observe')
    cg.write_pose(bus, poses['observe'], args.move_time)
    time.sleep(0.8)

    print('[2/6] look：镜头朝下观察位，并张开夹爪')
    cg.write_pose(bus, poses['look'], args.move_time)
    time.sleep(1.0)

    raw = cg.read_positions(bus)
    raw[5] = args.grip_open
    cg.write_pose(bus, raw, 500)
    time.sleep(0.5)

    j1_offset = 0
    name, area = args.target_class, 0.0
    if args.align_pose == 'look':
        j1_offset, name, area = align_at(
            source, bus, detector, args, '3/6 look', poses['look'])

    print('[4/6] 移动到 pre')
    cg.write_pose(bus, pr.with_grip(
        pr.with_j1_offset(poses['pre'], j1_offset), args.grip_open),
        args.move_time)
    time.sleep(0.8)

    if args.align_pose == 'pre':
        j1_offset, name, area = align_at(
            source, bus, detector, args, '4/6 pre', poses['pre'])

    if args.dry_run:
        print('干跑：已到 pre 并完成对准，不执行 grasp/lift')
        return

    print('[5/6] grasp -> 夹紧')
    cg.write_pose(bus, pr.with_grip(
        pr.with_j1_offset(poses['grasp'], j1_offset), args.grip_open),
        args.grasp_time)
    time.sleep(0.8)

    raw = cg.read_positions(bus)
    raw[5] = args.grip_close
    cg.write_pose(bus, raw, args.close_time)
    time.sleep(args.close_time / 1000.0 + 0.3)

    print('[6/6] lift')
    cg.write_pose(bus, pr.with_grip(
        pr.with_j1_offset(poses['lift'], j1_offset), args.grip_close),
        args.lift_time)
    time.sleep(0.8)
    print(f'抓取完成：YOLO 触发类={name}，area={int(area)}')


if __name__ == '__main__':
    main()
