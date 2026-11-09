#!/usr/bin/env python3
"""YOLOv5 model0.pt detection helper for the RDK X5 arm camera.

The checkpoint is an old YOLOv5s model with 24 garbage classes.  It is not a
colour-block model.  Run this file from the board while the arm camera is being
served by mjpeg_server.py on port 8082:

    python3 ~/yolo_detect.py --save ~/yolo_result.jpg
    python3 ~/yolo_detect.py --source /dev/video0 --save ~/yolo_result.jpg
    python3 ~/yolo_detect.py --target-class Plastic_bottle --frames 5

The script intentionally keeps the detector separate from pick_rear.py so the
already working HSV grasp path is not disturbed.
"""

import argparse
import json
import os
import sys
import time
import types

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import color_grab as cg


# Class order stored in model0.pt.  The checkpoint itself also carries names.
DEFAULT_CLASSES = [
    'Zip_top_can',
    'Apple_core',
    'Banana_peel',
    'Bandage',
    'Basketball',
    'Book',
    'Carton',
    'Cigarette_butts',
    'Disposable_chopsticks',
    'Egg_shell',
    'Expired_capsule_drugs',
    'Expired_cosmetics',
    'Expired_tablets',
    'Fish_bone',
    'Newspaper',
    'Old_school_bag',
    'Peach_pit',
    'Plastic_bottle',
    'Shell',
    'Syringe',
    'Toilet_paper',
    'Used_batteries',
    'Vegetable_leaf',
    'Watermelon_rind',
]


# Fallback anchors for the standard yolov5s head used by model0.pt.
DEFAULT_ANCHORS = [
    [[10, 13], [16, 30], [33, 23]],
    [[30, 61], [62, 45], [59, 119]],
    [[116, 90], [156, 198], [373, 326]],
]


def letterbox(image, new_shape):
    """Resize and pad a BGR image to new_shape without changing aspect ratio."""
    height, width = image.shape[:2]
    ratio = min(new_shape[0] / height, new_shape[1] / width)
    new_unpad = (int(round(width * ratio)), int(round(height * ratio)))
    if (width, height) != new_unpad:
        image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
    dw = new_shape[1] - new_unpad[0]
    dh = new_shape[0] - new_unpad[1]
    left = int(round(dw / 2 - 0.1))
    right = int(round(dw / 2 + 0.1))
    top = int(round(dh / 2 - 0.1))
    bottom = int(round(dh / 2 + 0.1))
    image = cv2.copyMakeBorder(
        image, top, bottom, left, right,
        cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return image, ratio, (left, top)


def normalize_names(names):
    if isinstance(names, dict):
        return {int(key): str(value) for key, value in names.items()}
    if isinstance(names, (list, tuple)):
        return {index: str(value) for index, value in enumerate(names)}
    return {}


def legacy_detect_forward(self, x):
    """Decode an early YOLOv5 Detect head without its zeroed anchor buffer."""
    torch = self._rdx_torch
    z = []
    device = x[0].device
    dtype = x[0].dtype

    anchors = getattr(self, 'anchors', None)
    if anchors is None:
        anchors = torch.tensor(DEFAULT_ANCHORS, dtype=dtype, device=device)
    else:
        if not torch.is_tensor(anchors):
            anchors = torch.tensor(anchors, dtype=dtype, device=device)
        anchors = anchors.to(device=device, dtype=dtype)
        if anchors.numel() == 0 or float(anchors.abs().sum()) == 0.0:
            anchors = torch.tensor(DEFAULT_ANCHORS, dtype=dtype, device=device)
        else:
            try:
                anchors = anchors.reshape(self.nl, self.na, 2)
            except Exception:
                anchors = torch.tensor(DEFAULT_ANCHORS, dtype=dtype, device=device)

    stride = getattr(self, 'stride', None)
    if stride is None:
        stride = [8, 16, 32]
    if torch.is_tensor(stride):
        stride_values = stride.detach().cpu().tolist()
    else:
        stride_values = list(stride)
    if len(stride_values) < self.nl or min(float(v) for v in stride_values[:self.nl]) <= 0:
        stride_values = [8, 16, 32]

    for i in range(self.nl):
        x[i] = self.m[i](x[i])
        bs, _, ny, nx = x[i].shape
        x[i] = x[i].view(bs, self.na, self.no, ny, nx).permute(0, 1, 3, 4, 2).contiguous()

        if not self.training:
            try:
                yv, xv = torch.meshgrid(
                    torch.arange(ny, device=device, dtype=dtype),
                    torch.arange(nx, device=device, dtype=dtype),
                    indexing='ij')
            except TypeError:
                # Older PyTorch on RDK X5 has no indexing= keyword.
                yv, xv = torch.meshgrid(
                    torch.arange(ny, device=device, dtype=dtype),
                    torch.arange(nx, device=device, dtype=dtype))
            grid = torch.stack((xv, yv), 2).expand(1, self.na, ny, nx, 2) - 0.5
            anchor_grid = (anchors[i] * float(stride_values[i])).view(1, self.na, 1, 1, 2)
            y = x[i].sigmoid()
            xy = (y[..., 0:2] * 2.0 + grid) * float(stride_values[i])
            wh = (y[..., 2:4] * 2.0) ** 2 * anchor_grid
            out = torch.cat((xy, wh, y[..., 4:]), 4)
            z.append(out.view(bs, -1, self.no))

    return x if self.training else (torch.cat(z, 1), x)


def patch_legacy_model(model, torch):
    """Fill attributes added by newer YOLOv5 releases to old checkpoints."""
    for module in model.modules():
        if module.__class__.__name__ != 'Detect':
            continue
        module.dynamic = True
        if not hasattr(module, 'inplace'):
            module.inplace = True
        if not hasattr(module, 'export'):
            module.export = False
        module._rdx_torch = torch
        module.forward = types.MethodType(legacy_detect_forward, module)


class YoloDetector:
    """Small wrapper around either DetectMultiBackend or old attempt_load."""

    def __init__(self, weights, repo, torch_device='cpu', imgsz=416,
                 conf=0.25, iou=0.45):
        self.weights = os.path.abspath(os.path.expanduser(weights))
        self.repo = os.path.abspath(os.path.expanduser(repo))
        self.torch_device_name = torch_device
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        if not os.path.isfile(self.weights):
            raise SystemExit('找不到权重：' + self.weights)
        if not os.path.isdir(self.repo):
            raise SystemExit('找不到 YOLOv5 源码目录：' + self.repo)

        sys.path.insert(0, self.repo)
        import torch
        self.torch = torch
        self.device = torch.device(self.torch_device_name)
        self.model = None
        self.errors = []

        try:
            from models.common import DetectMultiBackend
            self.model = DetectMultiBackend(
                self.weights, device=self.device, fp16=False)
            stride = getattr(self.model, 'stride', 32)
            self.stride = int(stride) if not hasattr(stride, 'max') else int(stride.max())
        except Exception as exc:
            self.errors.append('DetectMultiBackend: ' + repr(exc))

        if self.model is None:
            try:
                from models.experimental import attempt_load
                self.model = attempt_load(self.weights, map_location=self.device)
                stride = getattr(self.model, 'stride', 32)
                self.stride = int(stride.max()) if hasattr(stride, 'max') else int(stride)
            except Exception as exc:
                self.errors.append('attempt_load: ' + repr(exc))

        if self.model is None:
            raise SystemExit('加载 model0.pt 失败：\n' + '\n'.join(self.errors))

        patch_legacy_model(self.model, self.torch)
        self.model.eval()
        try:
            self.model.float()
        except Exception:
            pass
        self.names = normalize_names(getattr(self.model, 'names', None))
        if not self.names:
            self.names = normalize_names(DEFAULT_CLASSES)
        from utils.general import non_max_suppression
        self.nms = non_max_suppression

    def class_name(self, class_id):
        return self.names.get(int(class_id), 'class_' + str(int(class_id)))

    def detect_frame(self, frame, targets=None, min_area=0.0):
        """Return detections as dicts, with coordinates in the original frame."""
        if frame is None:
            return []
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        original_h, original_w = frame.shape[:2]
        image, ratio, pad = letterbox(frame, (self.imgsz, self.imgsz))
        rgb = image[:, :, ::-1].transpose(2, 0, 1)
        tensor = np.ascontiguousarray(rgb)
        tensor = self.torch.from_numpy(tensor).to(self.device).float() / 255.0
        tensor = tensor.unsqueeze(0)

        with self.torch.no_grad():
            try:
                prediction = self.model(tensor, augment=False)
            except TypeError:
                prediction = self.model(tensor)
        if isinstance(prediction, (tuple, list)):
            prediction = prediction[0]
        detections = self.nms(prediction, self.conf, self.iou)[0]

        target_set = set()
        if targets:
            for target in targets:
                target = str(target).strip()
                if target and target.lower() != 'any':
                    target_set.add(target.lower())

        results = []
        for row in detections.tolist():
            x1, y1, x2, y2, confidence, class_id = row[:6]
            x1 = (x1 - pad[0]) / ratio
            x2 = (x2 - pad[0]) / ratio
            y1 = (y1 - pad[1]) / ratio
            y2 = (y2 - pad[1]) / ratio
            x1 = max(0.0, min(original_w - 1.0, x1))
            x2 = max(0.0, min(original_w - 1.0, x2))
            y1 = max(0.0, min(original_h - 1.0, y1))
            y2 = max(0.0, min(original_h - 1.0, y2))
            width = max(0.0, x2 - x1)
            height = max(0.0, y2 - y1)
            area = width * height
            if area < min_area:
                continue
            name = self.class_name(class_id)
            if target_set and name.lower() not in target_set \
                    and str(int(class_id)) not in target_set:
                continue
            results.append({
                'class': name,
                'class_id': int(class_id),
                'conf': float(confidence),
                'box': [int(round(x1)), int(round(y1)),
                        int(round(width)), int(round(height))],
                'cx': int(round(x1 + width / 2.0)),
                'cy': int(round(y1 + height / 2.0)),
                'area': float(area),
            })
        results.sort(key=lambda item: (item['conf'], item['area']), reverse=True)
        return results

    def find(self, frame, target_class='any', min_area=0.0):
        """Compatibility tuple: (name, area, cx, cy, box)."""
        if target_class in (None, '', 'any'):
            targets = None
        else:
            targets = [item.strip() for item in str(target_class).split(',')
                       if item.strip()]
        results = self.detect_frame(frame, targets, min_area)
        if not results:
            return None
        best = results[0]
        x, y, width, height = best['box']
        return best['class'], best['area'], best['cx'], best['cy'], (x, y, width, height)


def annotate(frame, results):
    output = frame.copy()
    for item in results:
        x, y, width, height = item['box']
        color = (0, 255, 0) if item is results[0] else (255, 160, 0)
        cv2.rectangle(output, (x, y), (x + width, y + height), color, 2)
        cv2.drawMarker(output, (item['cx'], item['cy']), color,
                       cv2.MARKER_CROSS, 20, 2)
        label = '{} {:.2f}'.format(item['class'], item['conf'])
        cv2.putText(output, label, (x, max(22, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    if not results:
        cv2.putText(output, 'NO TARGET', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return output


def ensure_parent(path):
    path = os.path.abspath(os.path.expanduser(path))
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return path


def parse_targets(text):
    if not text or text.strip().lower() == 'any':
        return None
    return [item.strip() for item in text.split(',') if item.strip()]


def build_parser():
    parser = argparse.ArgumentParser(description='model0.pt / YOLOv5 detection')
    parser.add_argument('--weights', default='~/model0.pt')
    parser.add_argument('--repo', default='~/yolov5-6.2',
                        help='YOLOv5 source directory, default ~/yolov5-6.2')
    parser.add_argument('--source', default='http://127.0.0.1:8082/snapshot',
                        help='mjpeg snapshot URL or /dev/video0')
    parser.add_argument('--camera-device', default='/dev/video0')
    parser.add_argument('--torch-device', default='cpu')
    parser.add_argument('--imgsz', type=int, default=416)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--min-area', type=float, default=0.0)
    parser.add_argument('--target-class', default='any',
                        help='one class name or comma-separated names; any = all')
    parser.add_argument('--frames', type=int, default=1)
    parser.add_argument('--interval', type=float, default=0.2)
    parser.add_argument('--save', default='~/yolo_result.jpg')
    parser.add_argument('--json', default='')
    parser.add_argument('--require', action='store_true',
                        help='exit with status 2 when no target is found')
    parser.add_argument('--list-classes', action='store_true')
    parser.add_argument('--debug', action='store_true')
    return parser


def main():
    args = build_parser().parse_args()
    detector = YoloDetector(
        args.weights, args.repo, args.torch_device, args.imgsz,
        args.conf, args.iou)
    if args.debug:
        for module in detector.model.modules():
            if module.__class__.__name__ == 'Detect':
                print('DEBUG Detect stride={} anchors={} grid_type={}'.format(
                    getattr(module, 'stride', None),
                    getattr(module, 'anchors', None).shape
                    if detector.torch.is_tensor(getattr(module, 'anchors', None))
                    else getattr(module, 'anchors', None),
                    type(getattr(module, 'grid', None)).__name__))
    if args.list_classes:
        for class_id, name in sorted(detector.names.items()):
            print('{} {}'.format(class_id, name))
        return

    source = cg.FrameSource(args.source, args.camera_device)
    targets = parse_targets(args.target_class)
    best_frame = None
    best_results = []
    best_item = None
    for index in range(max(1, args.frames)):
        frame = source.read()
        if frame is None:
            print('[{}] 取图失败'.format(index + 1))
            time.sleep(args.interval)
            continue
        results = detector.detect_frame(frame, targets, args.min_area)
        print('[{}] {} detection(s){}'.format(
            index + 1, len(results),
            ': ' + ', '.join('{} {:.2f} cx={} cy={}'.format(
                item['class'], item['conf'], item['cx'], item['cy'])
                for item in results) if results else ''))
        if results and (best_item is None or results[0]['conf'] > best_item['conf']):
            best_frame = frame
            best_results = results
            best_item = results[0]
        if index + 1 < args.frames:
            time.sleep(args.interval)

    if best_frame is None:
        print('没有取到画面')
        return 2 if args.require else 1

    save_path = ensure_parent(args.save)
    cv2.imwrite(save_path, annotate(best_frame, best_results))
    print('标注图：' + save_path)
    if args.json:
        json_path = ensure_parent(args.json)
        with open(json_path, 'w', encoding='utf-8') as handle:
            json.dump(best_results, handle, ensure_ascii=False, indent=2)
            handle.write('\n')
        print('检测 JSON：' + json_path)

    if best_item is None:
        print('RESULT none')
        return 2 if args.require else 0
    print('RESULT {} conf={:.3f} cx={} cy={} box={}'.format(
        best_item['class'], best_item['conf'], best_item['cx'],
        best_item['cy'], best_item['box']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
