#!/usr/bin/env python3
"""测试单个舵机是否有响应。

默认测 J6 夹爪。脚本只发一次小角度再回原位，不会连续堵转。
运行前先停 slave_joint_node.py，并保持手在 12V 开关旁。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import color_grab as cg


def main():
    parser = argparse.ArgumentParser(description='单舵机响应测试')
    parser.add_argument('--joint', type=int, default=6, help='1~6，默认 6')
    parser.add_argument('--step', type=int, default=120, help='测试幅度，默认 120')
    parser.add_argument('--wait', type=float, default=0.9, help='等待时间秒')
    args = parser.parse_args()

    if not 1 <= args.joint <= 6:
        raise SystemExit('--joint 只能是 1~6')

    index = args.joint - 1
    bus = cg.open_bus()
    raw = cg.read_positions(bus)
    start = raw[index]
    low = cg.RAW_MIN[index] + cg.MARGIN
    high = cg.RAW_MAX[index] - cg.MARGIN
    mid = (low + high) / 2

    if start >= mid:
        target = max(low, start - args.step)
    else:
        target = min(high, start + args.step)

    print(f'J{args.joint} 起始：{start}，范围 [{low}, {high}]')
    print(f'测试目标：{target}，回读等待 {args.wait}s')
    print('如果听到咔咔声或发烫，立即关 12V。')

    test = list(raw)
    test[index] = target
    cg.write_pose(bus, test, 450)
    time.sleep(args.wait)
    now = cg.read_positions(bus)
    actual = now[index]
    moved = abs(actual - start)

    print(f'目标：{target}')
    print(f'实际：{actual}')
    print(f'相对起始移动：{moved}')

    back = list(now)
    back[index] = start
    cg.write_pose(bus, back, 450)
    time.sleep(args.wait)
    final = cg.read_positions(bus)[index]
    print(f'回原位实际：{final}')

    if moved >= 25:
        print('结果：舵机有响应。若仍然咔咔响，检查齿轮、异物和舵机臂螺丝。')
    else:
        print('结果：舵机没有响应。优先查 12V 舵机供电、J6 线束和舵机本体。')


if __name__ == '__main__':
    main()
