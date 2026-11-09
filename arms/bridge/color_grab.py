#!/usr/bin/env python3
"""从臂颜色识别抓取 V1（无 ROS 依赖）。

运行在板A。扩展板 I2C5 / 0x15，协议与 slave_joint_node.py 相同。
运行前必须先停 slave_joint_node.py，两个进程不能同时写 I2C。

常用流程：
    python3 ~/color_grab.py read
    python3 ~/color_grab.py jog --joint 1 --delta 80
    python3 ~/color_grab.py save --name observe
    python3 ~/color_grab.py grip --open
    python3 ~/color_grab.py detect --color red
    python3 ~/color_grab.py preset --name look_at
    python3 ~/color_grab.py auto --color red
    python3 ~/color_grab.py grab --color red

位姿文件默认 ~/grab_poses.json，由 save 生成。抓取需要 observe/pre/grasp/lift
四个 6 路 raw 数组，缺一个都会拒绝执行。
"""

import argparse
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

try:
    import smbus
except ImportError:
    smbus = None


ADDR = 0x15
BUS = 5
RAW_MIN = [900, 900, 900, 900, 380, 900]
RAW_MAX = [3100, 3100, 3100, 3100, 3700, 3100]
MARGIN = 40
POSE_PATH = os.path.expanduser('~/grab_poses.json')

GRIP_OPEN = 1633
GRIP_CLOSE = 2489

# Final demo: arm camera; official color_grab checks only the center ROI.
ARM_SOURCE = 'http://127.0.0.1:8082/snapshot'
ARM_DEVICE = '/dev/video0'
OFFICIAL_ROI = '280,180,360,260'

# 官方 Dofbot 教程 6.AI_Visual/3.color_grab.ipynb 的位姿，单位：度。
# 顺序 J1~J5，J6 夹爪单独控制。
PRESETS_DEG = {
    'look_at': [90, 164, 18, 0, 90],
    'top': [90, 80, 50, 50, 270],
    'gray': [90, 48, 35, 30, 270],
    'red': [118, 19, 66, 56, 270],
    'green': [136, 66, 20, 29, 270],
    'blue': [44, 66, 20, 28, 270],
    'yellow': [65, 22, 64, 56, 270],
}

HSV = {
    'red': [((0, 43, 46), (10, 255, 255)), ((156, 43, 46), (180, 255, 255))],
    'green': [((35, 43, 46), (78, 255, 255))],
    'blue': [((100, 43, 46), (124, 255, 255))],
    'yellow': [((26, 43, 46), (34, 255, 255))],
}
BGR = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
}


def open_bus():
    if smbus is None:
        sys.exit('缺少 smbus：sudo apt install -y python3-smbus')
    try:
        return smbus.SMBus(BUS)
    except Exception as exc:
        sys.exit(f'打不开 /dev/i2c-{BUS}：{exc}（检查接线、共地、权限）')


def read_positions(bus):
    values = []
    for offset in range(6):
        reg = 0x31 + offset
        value = None
        last_error = None
        for _ in range(3):
            try:
                bus.write_byte_data(ADDR, reg, 0x00)
                time.sleep(0.003)
                raw = bus.read_word_data(ADDR, reg)
                raw = ((raw >> 8) & 0xFF) | ((raw << 8) & 0xFF00)
                if 96 <= raw <= 4000:
                    value = raw
                    break
                last_error = f'读数异常 {raw}'
            except Exception as exc:
                last_error = exc
            time.sleep(0.05)
        if value is None:
            sys.exit(
                f'读取 J{offset + 1} 失败：{last_error}（杜邦线/共地/12V）')
        values.append(value)
    return values


def clamp_pose(raw):
    out = []
    for i, value in enumerate(raw[:6]):
        low = RAW_MIN[i] + MARGIN
        high = RAW_MAX[i] - MARGIN
        value = int(round(value))
        if value < low or value > high:
            print(f'警告：J{i + 1} {value} 超限，夹到 [{low}, {high}]')
            value = max(low, min(high, value))
        out.append(value)
    return out


def write_pose(bus, raw, time_ms=800):
    raw = clamp_pose(raw)
    data = []
    for value in raw:
        value = int(max(96, min(4000, value)))
        data.extend([(value >> 8) & 0xFF, value & 0xFF])
    last_error = None
    for _ in range(3):
        try:
            bus.write_i2c_block_data(
                ADDR, 0x1E, [(time_ms >> 8) & 0xFF, time_ms & 0xFF])
            bus.write_i2c_block_data(ADDR, 0x1D, data)
            return raw
        except Exception as exc:
            last_error = exc
            time.sleep(0.05)
    sys.exit(f'I2C 写入失败：{last_error}（杜邦线/共地/12V）')


def fmt_pose(raw):
    return ', '.join(f'J{i + 1}={v}' for i, v in enumerate(raw))


def to_deg(index, raw):
    if index == 4:
        return (raw - RAW_MIN[4]) * 270.0 / (RAW_MAX[4] - RAW_MIN[4])
    return (raw - RAW_MIN[index]) * 180.0 / (RAW_MAX[index] - RAW_MIN[index])


def deg_to_raw(index, deg):
    """与官方 Arm_Lib.Arm_serial_servo_write 完全一致。"""
    deg = float(deg)
    if index == 4:
        deg = max(0.0, min(270.0, deg))
        return int(380 + 3320.0 * deg / 270.0)
    deg = max(0.0, min(180.0, deg))
    if index in (1, 2, 3):
        deg = 180.0 - deg
    return int(900 + 2200.0 * deg / 180.0)


def deg_pose(deg_values, current):
    raw = list(current)
    for index, deg in enumerate(deg_values[:6]):
        raw[index] = deg_to_raw(index, deg)
    return raw


def parse_roi(text):
    if not text or text.lower() in ('full', 'none'):
        return None
    try:
        parts = [int(value) for value in text.split(',')]
    except ValueError:
        sys.exit('--roi 格式：x0,y0,x1,y1')
    if len(parts) != 4:
        sys.exit('--roi 必须 4 个数：x0,y0,x1,y1')
    return tuple(parts)


def load_poses():
    if not os.path.exists(POSE_PATH):
        return {}
    try:
        with open(POSE_PATH, 'r', encoding='utf-8') as handle:
            return json.load(handle)
    except Exception as exc:
        sys.exit(f'读 {POSE_PATH} 失败：{exc}')


def save_poses(data):
    tmp = POSE_PATH + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as handle:
        json.dump(data, handle, indent=2)
        handle.write('\n')
    os.replace(tmp, POSE_PATH)


class FrameSource:
    def __init__(self, source, device):
        self.source = source
        self.device = device
        self.cap = None

    def read(self):
        if self.source.startswith('http://') or self.source.startswith('https://'):
            try:
                with urllib.request.urlopen(self.source, timeout=2.0) as resp:
                    data = resp.read()
                frame = cv2.imdecode(
                    np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    print(f'{self.source} 返回的不是 JPEG')
                return frame
            except Exception as exc:
                print(f'取图失败：{exc}（确认图传 {self.source} 在跑）')
                return None

        if self.cap is None:
            device = self.source if self.source.startswith('/dev/') else self.device
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.cap.isOpened():
                print(f'打不开 {device}（是否被图传占用？）')
        ok, frame = self.cap.read()
        return frame if ok else None


def detect(frame, color, min_area, roi=None):
    ox = oy = 0
    view = frame
    if roi is not None:
        height, width = frame.shape[:2]
        x0, y0, x1, y1 = roi
        x0 = max(0, min(width, x0))
        x1 = max(0, min(width, x1))
        y0 = max(0, min(height, y0))
        y1 = max(0, min(height, y1))
        if x1 <= x0 or y1 <= y0:
            return None
        view = frame[y0:y1, x0:x1]
        ox, oy = x0, y0

    hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)
    names = list(HSV) if color == 'any' else [color]
    best = None
    for name in names:
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in HSV[name]:
            mask |= cv2.inRange(hsv, np.array(low), np.array(high))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        found = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = found[0] if len(found) == 2 else found[1]
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if h == 0 or w / float(h) < 0.25 or w / float(h) > 4.0:
            continue
        if best is None or area > best[1]:
            best = (name, area, ox + x + w // 2, oy + y + h // 2,
                    (ox + x, oy + y, w, h))
    return best


def annotate(frame, found):
    if found is None:
        cv2.putText(frame, 'NO TARGET', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return frame
    name, area, cx, cy, (x, y, w, h) = found
    cv2.rectangle(frame, (x, y), (x + w, y + h), BGR[name], 2)
    cv2.drawMarker(frame, (cx, cy), BGR[name], cv2.MARKER_CROSS, 24, 2)
    cv2.putText(frame, f'{name} area={int(area)} cx={cx} cy={cy}',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, BGR[name], 2)
    return frame


def detect_stable(source, color, frames, interval, min_area, save_path, roi=None):
    votes = {}
    accepted = None
    last = None
    for index in range(frames):
        frame = source.read()
        if frame is None:
            return None
        last = frame
        found = detect(frame, color, min_area, roi)
        if found is not None:
            votes[found[0]] = votes.get(found[0], 0) + 1
            accepted = found
            print(f'[{index + 1}/{frames}] {found[0]} '
                  f'area={int(found[1])} cx={found[2]} cy={found[3]}')
        else:
            print(f'[{index + 1}/{frames}] 未识别到')
        time.sleep(interval)

    if not votes:
        print('整段画面没有目标颜色')
        return None

    target = color if color != 'any' else max(votes, key=votes.get)
    need = max(2, int(frames * 0.6))
    if votes.get(target, 0) < need:
        print(f'颜色不稳定：{votes}，要求 {target} >= {need}/{frames}')
        return None
    if accepted is None or accepted[0] != target:
        print(f'最后一帧不是 {target}，判为不稳定')
        return None

    if save_path and last is not None:
        cv2.imwrite(save_path, annotate(last.copy(), accepted))
        print(f'标注图已存：{save_path}')
    return accepted


def cmd_read(args):
    bus = open_bus()
    raw = read_positions(bus)
    print(fmt_pose(raw))
    print(', '.join(f'J{i + 1}={to_deg(i, v):.1f}deg' for i, v in enumerate(raw)))


def cmd_jog(args):
    if not 1 <= args.joint <= 6:
        sys.exit('--joint 只能是 1~6')
    bus = open_bus()
    raw = read_positions(bus)
    target = list(raw)
    target[args.joint - 1] += args.delta
    target = write_pose(bus, target, args.time)
    time.sleep(args.time / 1000.0 + args.settle)
    now = read_positions(bus)
    print(f'目标：{fmt_pose(target)}')
    print(f'实际：{fmt_pose(now)}')


def cmd_save(args):
    bus = open_bus()
    raw = read_positions(bus)
    data = load_poses()
    data[args.name] = raw
    save_poses(data)
    print(f'已保存 {args.name}: {fmt_pose(raw)}')
    print(f'文件：{POSE_PATH}')
    missing = [name for name in ('observe', 'pre', 'grasp', 'lift')
               if not data.get(name)]
    print('还缺：' + ', '.join(missing) if missing else '四个位姿齐全')


def cmd_move(args):
    try:
        raw = [int(part) for part in args.raw.split(',')]
    except ValueError:
        sys.exit('--raw 格式：1633,1800,1500,1500,2000,1633')
    if len(raw) != 6:
        sys.exit('--raw 必须 6 个数')
    if args.dry_run:
        print(f'干跑：{fmt_pose(clamp_pose(raw))}')
        return
    bus = open_bus()
    write_pose(bus, raw, args.time)
    time.sleep(args.time / 1000.0 + args.settle)
    print(f'实际：{fmt_pose(read_positions(bus))}')


def cmd_grip(args):
    if args.raw is not None:
        value = args.raw
    elif args.open:
        value = GRIP_OPEN
    else:
        value = GRIP_CLOSE
    bus = open_bus()
    raw = read_positions(bus)
    raw[5] = value
    raw = write_pose(bus, raw, args.time)
    time.sleep(args.time / 1000.0 + args.settle)
    print(f'夹爪 raw={raw[5]}，实际={read_positions(bus)[5]}')


def cmd_preset(args):
    if args.name not in PRESETS_DEG:
        sys.exit('可选：' + ', '.join(PRESETS_DEG))
    bus = open_bus()
    current = read_positions(bus)
    target = deg_pose(PRESETS_DEG[args.name], current)
    write_pose(bus, target, args.time)
    time.sleep(args.time / 1000.0 + args.settle)
    print(f'{args.name} 目标：{fmt_pose(target)}')
    print(f'实际：{fmt_pose(read_positions(bus))}')


def cmd_auto(args):
    """官方 color_grab 流程：look_at -> 识色 -> top -> 颜色位 -> 夹紧。"""
    source = FrameSource(args.source, args.device)
    roi = parse_roi(args.roi)

    if args.dry_run:
        print('干跑：只识色，不动从臂')
        found = detect_stable(
            source, args.color, args.frames, args.interval,
            args.min_area, args.save, roi)
        if found is None:
            sys.exit(2)
        color = found[0]
        print(f'识别成功 {color}，正式运行会依次执行：')
        print(f'  look_at {PRESETS_DEG["look_at"]}')
        print(f'  top     {PRESETS_DEG["top"]}')
        print(f'  {color:<7} {PRESETS_DEG[color]}')
        if not args.no_drop:
            print(f'  gray    {PRESETS_DEG["gray"]}')
        return

    bus = open_bus()
    log_dir = os.path.expanduser('~/grab_logs')
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')

    print(f'[观察位] {PRESETS_DEG["look_at"]}')
    current = read_positions(bus)
    write_pose(bus, deg_pose(PRESETS_DEG['look_at'], current), args.move_time)
    time.sleep(args.settle + 0.5)

    print('[识色] 中央 ROI')
    found = detect_stable(
        source, args.color, args.frames, args.interval, args.min_area,
        os.path.join(log_dir, f'{stamp}_detect.jpg'), roi)
    if found is None:
        sys.exit('识别失败，未执行抓取')
    color = found[0]
    if color not in PRESETS_DEG:
        sys.exit(f'没有 {color} 的预设位姿')

    print(f'[顶部位] {PRESETS_DEG["top"]}')
    current = read_positions(bus)
    current[5] = args.grip_open
    write_pose(bus, deg_pose(PRESETS_DEG['top'], current), args.move_time)
    time.sleep(args.settle)

    print(f'[{color} 抓取位] {PRESETS_DEG[color]}')
    current = read_positions(bus)
    write_pose(bus, deg_pose(PRESETS_DEG[color], current), args.move_time)
    time.sleep(args.settle)

    print(f'[夹紧] raw={args.grip_close}')
    current = read_positions(bus)
    current[5] = args.grip_close
    write_pose(bus, current, 700)
    time.sleep(0.8)

    print(f'[抬起] {PRESETS_DEG["top"]}')
    current = read_positions(bus)
    write_pose(bus, deg_pose(PRESETS_DEG['top'], current), args.move_time)
    time.sleep(args.settle)

    if not args.no_drop:
        print(f'[放置位] {PRESETS_DEG["gray"]}')
        current = read_positions(bus)
        write_pose(bus, deg_pose(PRESETS_DEG['gray'], current), args.move_time)
        time.sleep(args.settle)
        current = read_positions(bus)
        current[5] = args.grip_open
        write_pose(bus, current, 600)
        time.sleep(0.6)
        print('[松爪] 完成')

    print(f'[回观察位] {PRESETS_DEG["look_at"]}')
    current = read_positions(bus)
    write_pose(bus, deg_pose(PRESETS_DEG['look_at'], current), args.move_time)
    time.sleep(args.settle)
    print(f'抓取完成：{color}')
    print(f'标注图：{log_dir}/{stamp}_detect.jpg')


def cmd_detect(args):
    source = FrameSource(args.source, args.device)
    found = detect_stable(
        source, args.color, args.frames, args.interval,
        args.min_area, args.save, parse_roi(args.roi))
    if found is None:
        sys.exit(2)
    print(f'RESULT {found[0]} area={int(found[1])} '
          f'cx={found[2]} cy={found[3]}')


def cmd_grab(args):
    poses = load_poses()
    missing = [name for name in ('observe', 'pre', 'grasp', 'lift')
               if not isinstance(poses.get(name), list)
               or len(poses[name]) != 6]
    if missing:
        sys.exit('缺少位姿：' + ', '.join(missing)
                 + '。先 jog 到位，再 save --name 对应名字')

    source = FrameSource(args.source, args.device)
    if args.dry_run:
        print('干跑：只识色，不动从臂')
        found = detect_stable(
            source, args.color, args.frames, args.interval,
            args.min_area, args.save, parse_roi(args.roi))
        if found is None:
            sys.exit(2)
        print(f'识别成功 {found[0]}，正式运行会依次执行：')
        print(f'  observe {fmt_pose(poses["observe"])}')
        print(f'  pre     {fmt_pose(poses["pre"])}')
        print(f'  grasp   {fmt_pose(poses["grasp"])}')
        print(f'  lift    {fmt_pose(poses["lift"])}')
        return

    bus = open_bus()
    log_dir = os.path.expanduser('~/grab_logs')
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')

    print(f'[1/6] 移动到观察位 {fmt_pose(poses["observe"])}')
    write_pose(bus, poses['observe'], args.move_time)
    time.sleep(args.settle + 0.5)

    print('[2/6] 识别颜色')
    found = detect_stable(
        source, args.color, args.frames, args.interval,
        args.min_area, os.path.join(log_dir, f'{stamp}_detect.jpg'),
        parse_roi(args.roi))
    if found is None:
        sys.exit('识别失败，未执行抓取')
    if args.color != 'any' and found[0] != args.color:
        sys.exit(f'目标是 {args.color}，实际看到 {found[0]}，不抓')

    print(f'[3/6] 张开夹爪 raw={args.grip_open}')
    raw = read_positions(bus)
    raw[5] = args.grip_open
    write_pose(bus, raw, 600)
    time.sleep(0.6)

    print(f'[4/6] 预抓取位 {fmt_pose(poses["pre"])}')
    write_pose(bus, poses['pre'], args.move_time)
    time.sleep(args.settle)

    print(f'[5/6] 抓取位 {fmt_pose(poses["grasp"])}')
    write_pose(bus, poses['grasp'], args.grasp_time)
    time.sleep(args.settle)

    raw = read_positions(bus)
    print(f'夹紧 raw={args.grip_close}')
    raw[5] = args.grip_close
    write_pose(bus, raw, 700)
    time.sleep(0.8)

    print(f'[6/6] 抬升 {fmt_pose(poses["lift"])}')
    write_pose(bus, poses['lift'], args.lift_time)
    time.sleep(args.settle)

    print(f'抓取完成：{found[0]}')
    print(f'标注图：{log_dir}/{stamp}_detect.jpg')


def build_parser():
    parser = argparse.ArgumentParser(description='从臂颜色识别抓取 V1')
    sub = parser.add_subparsers(dest='mode', required=True)

    sub.add_parser('read', help='读六路当前 raw')

    jog = sub.add_parser('jog', help='单关节点动')
    jog.add_argument('--joint', type=int, required=True)
    jog.add_argument('--delta', type=int, required=True)
    jog.add_argument('--time', type=int, default=400)
    jog.add_argument('--settle', type=float, default=0.4)

    save = sub.add_parser('save', help='把当前位姿存进 grab_poses.json')
    save.add_argument('--name', required=True)

    move = sub.add_parser('move', help='移动到六个 raw')
    move.add_argument('--raw', required=True)
    move.add_argument('--time', type=int, default=1200)
    move.add_argument('--settle', type=float, default=0.5)
    move.add_argument('--dry-run', action='store_true')

    grip = sub.add_parser('grip', help='只动夹爪')
    group = grip.add_mutually_exclusive_group(required=True)
    group.add_argument('--open', action='store_true')
    group.add_argument('--close', action='store_true')
    group.add_argument('--raw', type=int)
    grip.add_argument('--time', type=int, default=700)
    grip.add_argument('--settle', type=float, default=0.6)

    preset = sub.add_parser('preset', help='移动到官方位姿')
    preset.add_argument('--name', required=True,
                        choices=list(PRESETS_DEG))
    preset.add_argument('--time', type=int, default=1500)
    preset.add_argument('--settle', type=float, default=0.5)

    for name, func, help_text in (
            ('detect', cmd_detect, '只识色不动臂'),
            ('grab', cmd_grab, '识别并执行自定义预设抓取'),
            ('auto', cmd_auto, '按官方位姿自动识色抓取')):
        is_auto = name == 'auto'
        cmd = sub.add_parser(name, help=help_text)
        cmd.add_argument('--color', default='any',
                         choices=['any', 'red', 'green', 'blue', 'yellow'])
        cmd.add_argument('--frames', type=int, default=10)
        cmd.add_argument('--interval', type=float, default=0.12)
        cmd.add_argument(
            '--min-area', type=float, default=300.0 if is_auto else 600.0)
        cmd.add_argument('--save', default='')
        cmd.add_argument(
            '--source',
            default=ARM_SOURCE if is_auto else 'http://127.0.0.1:8081/snapshot')
        cmd.add_argument(
            '--device', default=ARM_DEVICE if is_auto else '/dev/video2')
        cmd.add_argument(
            '--roi',
            default=OFFICIAL_ROI if is_auto else 'full',
            help='x0,y0,x1,y1；full 表示整幅画面')
        cmd.set_defaults(func=func)
        if name in ('grab', 'auto'):
            cmd.add_argument('--dry-run', action='store_true')
            cmd.add_argument('--move-time', type=int, default=1500)
            cmd.add_argument('--grasp-time', type=int, default=900)
            cmd.add_argument('--lift-time', type=int, default=1200)
            cmd.add_argument('--settle', type=float, default=0.8)
            cmd.add_argument('--grip-open', type=int, default=GRIP_OPEN)
            cmd.add_argument('--grip-close', type=int, default=GRIP_CLOSE)
        if name == 'auto':
            cmd.add_argument('--no-drop', action='store_true')

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    funcs = {
        'read': cmd_read,
        'jog': cmd_jog,
        'save': cmd_save,
        'move': cmd_move,
        'grip': cmd_grip,
        'preset': cmd_preset,
        'detect': cmd_detect,
        'grab': cmd_grab,
        'auto': cmd_auto,
    }
    funcs[args.mode](args)


if __name__ == '__main__':
    main()
