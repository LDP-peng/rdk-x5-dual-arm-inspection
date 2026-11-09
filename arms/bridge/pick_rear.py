#!/usr/bin/env python3
"""后置机械臂视觉抓取：observe -> look -> HSV 对准 -> pre -> grasp -> lift。

前提：
    1. slave_joint_node.py 已停止；
    2. 机械臂相机在 8082 图传：mjpeg_server.py --device /dev/video0 --port 8082；
    3. grab_poses.json 已保存 observe/look/pre/grasp/lift。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg


def require_poses():
    data = cg.load_poses()
    need = ('observe', 'look', 'pre', 'grasp', 'lift')
    missing = [name for name in need
               if not isinstance(data.get(name), list)
               or len(data[name]) != 6]
    if missing:
        raise SystemExit('缺少位姿：' + ', '.join(missing)
                         + '。先用 arm_teach.py 示教并 save。')
    return data


def get_target(source, color, min_area, tries=4):
    for _ in range(tries):
        frame = source.read()
        if frame is not None:
            found = cg.detect(frame, color, min_area)
            if found is not None:
                return found
        time.sleep(0.12)
    return None


def jog_j1(bus, delta, wait=0.45):
    raw = cg.read_positions(bus)
    raw[0] += delta
    cg.write_pose(bus, raw, 420)
    time.sleep(wait)


def with_j1_offset(pose, offset):
    result = list(pose)
    result[0] += offset
    return result


def with_grip(pose, value):
    result = list(pose)
    result[5] = value
    return result


def align_at(source, bus, args, label, pose):
    print(f'[{label}] 机械臂相机识别颜色')
    found = get_target(source, args.color, args.min_area)
    if found is None:
        if not args.fixed:
            raise SystemExit(f'{label} 没有看到目标颜色：检查 8082 图传/积木位置')
        print('  未识别到目标颜色，固定位姿继续抓取')
        return 0, args.color, 0.0

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
        found = get_target(source, args.color, args.min_area)
        if found is None:
            raise SystemExit('对准过程中目标丢失，停止')
        name, area, cx, cy, _ = found
        error = cx - args.target_cx
        print(f'  try={index + 1} cx={cx} err={error:.0f}')
        if abs(error) <= args.center_tol:
            print('  已对准')
            break

        delta = int(0.5 * error)
        if delta == 0:
            delta = 40 if error > 0 else -40
        delta = max(-args.max_j1_step,
                    min(args.max_j1_step, delta)) * args.align_sign
        jog_j1(bus, delta)
    else:
        raise SystemExit('J1 对准失败：可尝试 --align-sign -1')

    aligned = cg.read_positions(bus)
    j1_offset = aligned[0] - pose[0]
    print(f'  J1 对准偏移：{j1_offset}（后续位姿会跟随）')
    return j1_offset, name, area


def main():
    parser = argparse.ArgumentParser(description='后置机械臂 HSV 抓取')
    parser.add_argument('--color', default='blue')
    parser.add_argument('--min-area', type=float, default=300.0)
    parser.add_argument('--center-tol', type=float, default=35.0)
    parser.add_argument('--max-j1-step', type=int, default=100)
    parser.add_argument('--max-align-tries', type=int, default=8)
    parser.add_argument('--align-sign', type=int, choices=(1, -1), default=1,
                        help='J1 修正方向；第一次方向反了就改 -1')
    parser.add_argument('--camera', default='http://127.0.0.1:8082/snapshot')
    parser.add_argument('--move-time', type=int, default=1200)
    parser.add_argument('--grasp-time', type=int, default=900)
    parser.add_argument('--lift-time', type=int, default=1200)
    parser.add_argument('--grip-open', type=int, default=cg.GRIP_OPEN,
                        help='夹爪张开 raw，越小张得越大')
    parser.add_argument('--grip-close', type=int, default=cg.GRIP_CLOSE,
                        help='夹紧 raw，越大越紧；注意舵机发热')
    parser.add_argument('--close-time', type=int, default=900,
                        help='夹紧动作持续时间 ms')
    parser.add_argument('--fixed', action='store_true',
                        help='固定工位抓取：不做J1视觉对准，识别不到也继续')
    parser.add_argument('--target-cx', type=float, default=320.0,
                        help='视觉对准的目标像素列，默认画面中心320')
    parser.add_argument('--align-pose', choices=('look', 'pre'), default='pre',
                        help='在哪个位姿做视觉对准；pre 离积木更近，默认 pre')
    parser.add_argument('--calibrate', action='store_true',
                        help='只识别并打印 cx，不抓取，用于标定 --target-cx')
    args = parser.parse_args()

    poses = require_poses()
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
    name, area = args.color, 0.0
    if args.align_pose == 'look':
        j1_offset, name, area = align_at(
            source, bus, args, '3/6 look', poses['look'])

    print('[4/6] 移动到 pre')
    cg.write_pose(bus, with_grip(
        with_j1_offset(poses['pre'], j1_offset), args.grip_open), args.move_time)
    time.sleep(0.8)

    if args.align_pose == 'pre':
        j1_offset, name, area = align_at(
            source, bus, args, '4/6 pre', poses['pre'])

    print('[5/6] grasp -> 夹紧')
    cg.write_pose(bus, with_grip(
        with_j1_offset(poses['grasp'], j1_offset), args.grip_open), args.grasp_time)
    time.sleep(0.8)

    raw = cg.read_positions(bus)
    raw[5] = args.grip_close
    cg.write_pose(bus, raw, args.close_time)
    time.sleep(args.close_time / 1000.0 + 0.3)

    print('[6/6] lift')
    cg.write_pose(bus, with_grip(
        with_j1_offset(poses['lift'], j1_offset), args.grip_close),
        args.lift_time)
    time.sleep(0.8)
    print(f'抓取完成：{name}，area={int(area)}')


if __name__ == '__main__':
    main()
