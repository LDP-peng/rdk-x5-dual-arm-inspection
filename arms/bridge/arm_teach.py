#!/usr/bin/env python3
"""从臂示教模式：按键移动关节，保存固定抓取位姿。

运行前先停 slave_joint_node.py，避免两个进程同时写 I2C。
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg


def show(bus):
    raw = cg.read_positions(bus)
    print(cg.fmt_pose(raw))
    print('deg: ' + ', '.join(
        f'J{i + 1}={cg.to_deg(i, v):.1f}' for i, v in enumerate(raw)))
    return raw


def move(bus, joint, delta):
    raw = cg.read_positions(bus)
    raw[joint - 1] += delta
    target = cg.write_pose(bus, raw, 350)
    time.sleep(0.35)
    print(f'目标：{cg.fmt_pose(target)}')
    show(bus)


def set_grip(bus, value):
    raw = cg.read_positions(bus)
    raw[5] = value
    cg.write_pose(bus, raw, 500)
    time.sleep(0.5)
    show(bus)


def save_pose(bus, name):
    raw = cg.read_positions(bus)
    data = cg.load_poses()
    data[name] = raw
    cg.save_poses(data)
    print(f'已保存 {name}: {cg.fmt_pose(raw)}')
    print(f'文件：{cg.POSE_PATH}')


def go_pose(bus, name):
    data = cg.load_poses()
    if name not in data:
        print(f'没有保存过 {name}')
        return
    cg.write_pose(bus, data[name], 1200)
    time.sleep(1.2)
    show(bus)


def main():
    print('从臂示教模式')
    print('移动：1+ 1- ... 5+ 5-；改步长：step 80')
    print('夹爪：open / close')
    print('保存：save observe / save pre / save grasp / save lift')
    print('测试：pose observe；读数：read；退出：q')
    print('注意：保存 pre/grasp 前先 open；保存 lift 前先 close')
    bus = cg.open_bus()
    show(bus)
    step = 100
    while True:
        try:
            line = input('cmd> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in ('q', 'quit', 'exit'):
            break
        if line in ('r', 'read'):
            show(bus)
            continue
        if line in ('open', 'o'):
            set_grip(bus, cg.GRIP_OPEN)
            continue
        if line in ('close', 'c'):
            set_grip(bus, cg.GRIP_CLOSE)
            continue
        if line.startswith('step '):
            try:
                step = max(20, min(400, int(line.split()[1])))
                print(f'步长 {step}')
            except (IndexError, ValueError):
                print('用法：step 80')
            continue
        if len(line) == 2 and line[0] in '12345' and line[1] in '+-':
            joint = int(line[0])
            delta = step if line[1] == '+' else -step
            move(bus, joint, delta)
            continue
        if line.startswith('save '):
            save_pose(bus, line.split(None, 1)[1])
            continue
        if line.startswith('pose '):
            go_pose(bus, line.split(None, 1)[1])
            continue
        print('未知命令。可用：1+ 1- ... 5+ 5- / step 80 / open / close / save pre / pose pre / read / q')


if __name__ == '__main__':
    main()
