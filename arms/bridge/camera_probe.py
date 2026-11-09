#!/usr/bin/env python3
"""探测 USB 摄像头可用的 FOURCC/分辨率，并保存一帧。

运行前先停掉占用摄像头的 mjpeg_server.py。
"""

import os
import time

import cv2


DEVICE = '/dev/video0'
MODES = [
    ('MJPG', 1920, 1080),
    ('MJPG', 1280, 720),
    ('MJPG', 960, 540),
    ('MJPG', 640, 480),
    ('MJPG', 320, 240),
    ('YUYV', 1280, 720),
    ('YUYV', 960, 540),
    ('YUYV', 640, 480),
    ('YUYV', 320, 240),
]


def try_mode(fourcc_text, width, height):
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'{fourcc_text} {width}x{height}: 打不开')
        cap.release()
        return False

    fourcc = cv2.VideoWriter_fourcc(*fourcc_text)
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    time.sleep(0.25)

    frame = None
    for _ in range(12):
        ok, image = cap.read()
        if ok and image is not None:
            frame = image
            break
        time.sleep(0.08)

    if frame is None:
        print(f'{fourcc_text} {width}x{height}: 打开成功，但读不到帧')
        cap.release()
        return False

    path = os.path.expanduser(
        f'~/camera_probe_{fourcc_text}_{width}x{height}.jpg')
    cv2.imwrite(path, frame)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'{fourcc_text} {width}x{height}: OK，实际 '
          f'{actual_w}x{actual_h}，保存 {path}')
    cap.release()
    return True


def main():
    print(f'探测 {DEVICE}，先停掉 mjpeg_server.py')
    ok_modes = []
    for fourcc, width, height in MODES:
        if try_mode(fourcc, width, height):
            ok_modes.append((fourcc, width, height))

    if not ok_modes:
        print('没有可用模式：检查 USB 线、供电、摄像头是否被占用')
    else:
        print('可用模式：')
        for fourcc, width, height in ok_modes:
            print(f'  {fourcc} {width}x{height}')


if __name__ == '__main__':
    main()
