#!/usr/bin/env python3
"""板A：把 USB 相机变成浏览器可看的 MJPEG 流。

用法：
    python3 mjpeg_server.py --device /dev/video0 --port 8080

Windows 浏览器打开：
    http://<板A-IP>:8080/
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


latest = None
lock = threading.Lock()
frame_event = threading.Event()


def capture_loop(args):
    global latest

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        raise SystemExit(f'打不开相机 {args.device}')

    ok, frame = cap.read()
    if not ok:
        raise SystemExit(f'{args.device} 读不到画面，检查设备号/占用')

    print(f'{args.device} 已打开：{frame.shape[1]}x{frame.shape[0]}，'
          f'http://0.0.0.0:{args.port}/', flush=True)

    period = 1.0 / max(1, args.fps)
    frames = 0
    last = time.monotonic()

    while True:
        start = time.monotonic()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality]
        )
        if not ok:
            continue

        with lock:
            latest = buf.tobytes()
        frame_event.set()

        frames += 1
        now = time.monotonic()
        if now - last >= 5.0:
            print(f'{frames / (now - last):.1f} fps', flush=True)
            frames = 0
            last = now

        remain = period - (time.monotonic() - start)
        if remain > 0:
            time.sleep(remain)


PAGE = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>robot camera</title>
<style>
body{margin:0;background:#111;color:#ddd;font-family:sans-serif}
img{width:100%;height:auto;display:block}
p{margin:6px 10px;font-size:13px}
</style></head>
<body><p>robot camera <a href="/dual" style="color:#8cf">dual view</a></p><img src="/stream"></body></html>
"""


PAGE_DUAL = """<!doctype html>
<html><head><meta charset="utf-8"><title>robot dual camera</title>
<style>
html,body{margin:0;height:100%;background:#111;color:#ddd;font-family:sans-serif}
body{display:flex;flex-direction:column;height:100vh}
header{display:flex;gap:18px;align-items:center;padding:6px 12px;font-size:13px;background:#181818}
header a{color:#8cf;text-decoration:none}
header .hint{color:#777}
.grid{flex:1;min-height:0;display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:8px;box-sizing:border-box}
.panel{display:flex;flex-direction:column;min-width:0;min-height:0;background:#000;border:1px solid #333}
.panel h2{font-size:12px;font-weight:normal;margin:0;padding:4px 8px;background:#1b1b1b;color:#aaa}
.panel img{flex:1;min-height:0;width:100%;height:100%;object-fit:contain;display:block}
@media (max-width:900px){.grid{grid-template-columns:1fr;overflow:auto}.panel img{height:auto}}
</style></head>
<body>
<header><strong>Robot Dual Camera</strong><a href="/">single</a>
<span class="hint">left 8080 / right 8081</span>
<span id="cam2"></span></header>
<div class="grid">
<div class="panel"><h2>Main / 8080</h2><img src="/stream" alt="camera 1"></div>
<div class="panel"><h2>Second / 8081</h2><img id="stream2" alt="camera 2"></div>
</div>
<script>
const p = Number(window.location.port || 80);
const cam2 = window.location.protocol + "//" + window.location.hostname + ":" + (p + 1) + "/stream";
document.getElementById("stream2").src = cam2;
document.getElementById("cam2").textContent = cam2;
</script>
</body></html>
""".encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == '/dual':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(PAGE_DUAL)))
            self.end_headers()
            self.wfile.write(PAGE_DUAL)
        elif self.path == '/stream':
            self.stream()
        elif self.path == '/snapshot':
            with lock:
                data = latest
            if not data:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_error(404)

    def stream(self):
        self.send_response(200)
        self.send_header(
            'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
        )
        self.end_headers()
        try:
            while True:
                frame_event.wait(timeout=1.0)
                frame_event.clear()
                with lock:
                    data = latest
                if not data:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(
                    b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n'
                )
                self.wfile.write(data)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--quality', type=int, default=70)
    args = parser.parse_args()

    thread = threading.Thread(target=capture_loop, args=(args,), daemon=True)
    thread.start()

    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    server.daemon_threads = True
    print(f'listening on 0.0.0.0:{args.port}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
