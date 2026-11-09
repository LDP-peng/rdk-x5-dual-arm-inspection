#!/usr/bin/env python3
"""回放 arm_teach.py 保存的示教位姿。

位姿文件：~/grab_poses.json
运行前必须停止 slave_joint_node.py，两个进程不能同时写 I2C。

常用：
    python3 ~/arm_pose_demo.py --poses observe,pre,grasp,lift
    python3 ~/arm_pose_demo.py --poses pre,grasp,lift --no-return-home
    python3 ~/arm_pose_demo.py --dry-run
"""

import argparse
import subprocess
import sys
import time

import color_grab as cg


PREFERRED = ('observe', 'pre', 'grasp', 'lift')


def slave_is_running():
    try:
        result = subprocess.run(
            ['pgrep', '-af', 'slave_joint_node.py'],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0 and bool(result.stdout.strip())


def build_sequence(poses, requested):
    if requested and requested.lower() != 'auto':
        names = [name.strip() for name in requested.split(',') if name.strip()]
    else:
        names = [name for name in PREFERRED if name in poses]
        if not names:
            names = list(poses)

    missing = [name for name in names if name not in poses]
    if missing:
        raise ValueError(
            '示教文件中缺少位姿：' + ', '.join(missing)
            + f'；当前可用：{", ".join(poses) or "无"}'
        )
    if not names:
        raise ValueError('没有可回放的示教位姿')

    for name in names:
        if len(poses[name]) != 6:
            raise ValueError(f'位姿 {name} 不是 6 路数据')
    return names


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--poses', default='auto')
    parser.add_argument('--time-ms', type=int, default=1400)
    parser.add_argument('--settle', type=float, default=0.5)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--allow-concurrent', action='store_true')
    parser.add_argument('--no-return-home', action='store_false', dest='return_home')
    parser.set_defaults(return_home=True)
    args = parser.parse_args()

    if not args.dry_run and slave_is_running() and not args.allow_concurrent:
        sys.exit('slave_joint_node.py 正在运行，先停止它再回放示教位姿')

    poses = cg.load_poses()
    if not poses:
        sys.exit(f'没有示教位姿：{cg.POSE_PATH}')

    try:
        sequence = build_sequence(poses, args.poses)
    except ValueError as exc:
        sys.exit(str(exc))

    total = max(1, args.repeat) * len(sequence)
    move_time = max(200, args.time_ms)
    print(f'示教文件：{cg.POSE_PATH}')
    print('回放顺序：' + ' -> '.join(sequence)
          + (f'，重复 {args.repeat} 次' if args.repeat > 1 else ''))

    if args.dry_run:
        for name in sequence:
            print(f'{name}: {cg.fmt_pose(poses[name])}')
        return

    bus = cg.open_bus()
    start_pose = cg.read_positions(bus)
    print(f'起始位姿：{cg.fmt_pose(start_pose)}')

    index = 0
    try:
        for _ in range(max(1, args.repeat)):
            for name in sequence:
                index += 1
                print(f'[{index}/{total}] {name}: {cg.fmt_pose(poses[name])}')
                cg.write_pose(bus, poses[name], move_time)
                time.sleep(move_time / 1000.0 + max(0.0, args.settle))

        if args.return_home:
            print('回到起始位姿')
            cg.write_pose(bus, start_pose, move_time)
            time.sleep(move_time / 1000.0 + max(0.0, args.settle))
    except KeyboardInterrupt:
        print('\n回放中断，机械臂保持当前位姿')

    print('示教动作完成')


if __name__ == '__main__':
    main()
