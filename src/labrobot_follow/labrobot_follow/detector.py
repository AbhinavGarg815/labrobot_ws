"""Person detection with keypoints (YOLO pose, TensorRT) and re-ID embedding.

Both engines take uint8 RGB pixels; scaling/normalisation is inside the
graph (tools/export_follow_models.py), so the CPU work per frame is one copy
for the detector and a crop+resize per person for re-ID.
"""

import cv2
import numpy as np

from .trt_engine import TrtEngine


def nms_xyxy(boxes, scores, iou_thresh):
    """Plain greedy NMS. boxes (N,4) xyxy. Returns kept indices, best first."""
    order = np.argsort(-scores)
    keep = []
    x1, y1, x2, y2 = boxes.T
    area = (x2 - x1) * (y2 - y1)
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        iou = inter / np.maximum(area[i] + area[rest] - inter, 1e-9)
        order = rest[iou <= iou_thresh]
    return keep


def decode_pose_output(out, conf_thresh=0.15, iou_thresh=0.6, max_det=30):
    """Decode a YOLO pose head into a list of {box, score, kpts}.

    Accepts the two layouts Ultralytics exports:
      * (56, N): rows cx, cy, w, h, score, 17 x (x, y, conf) - needs NMS
        (YOLO11-pose; YOLO26-pose exported with end2end False);
      * (N, 57): x1, y1, x2, y2, score, class, 17 x 3 - NMS-free head.
    """
    out = np.asarray(out, dtype=np.float32)
    if out.ndim == 3:
        out = out[0]
    if out.shape[0] == 56:
        p = out.T
        p = p[p[:, 4] >= conf_thresh]
        if not len(p):
            return []
        cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
        boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
        scores = p[:, 4]
        kpts = p[:, 5:].reshape(-1, 17, 3)
        keep = nms_xyxy(boxes, scores, iou_thresh)[:max_det]
    elif out.ndim == 2 and out.shape[1] == 57:
        p = out[out[:, 4] >= conf_thresh]
        boxes, scores, kpts = p[:, :4], p[:, 4], p[:, 6:].reshape(-1, 17, 3)
        keep = list(np.argsort(-scores)[:max_det])
    else:
        raise ValueError(f'unknown pose output layout {out.shape}')
    return [{'box': boxes[i].copy(), 'score': float(scores[i]), 'kpts': kpts[i].copy()}
            for i in keep]


class PoseDetector:
    def __init__(self, engine_path, conf_thresh=0.15, iou_thresh=0.6):
        self.trt = TrtEngine(engine_path)
        self.in_name = self.trt.input_names[0]
        self.out_name = self.trt.output_names[0]
        _, self.h, self.w, _ = self.trt.input_shape(self.in_name)
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh

    def __call__(self, rgb):
        """rgb: HxWx3 uint8. Boxes/keypoints come back in rgb's pixel coords."""
        ih, iw = rgb.shape[:2]
        dst = self.trt.inputs[self.in_name][0]
        if (ih, iw) == (self.h, self.w):
            np.copyto(dst, rgb)
            sx = sy = 1.0
        else:
            cv2.resize(rgb, (self.w, self.h), dst=dst, interpolation=cv2.INTER_LINEAR)
            sx, sy = iw / self.w, ih / self.h
        out = self.trt.infer()[self.out_name]
        dets = decode_pose_output(out, self.conf_thresh, self.iou_thresh)
        if sx != 1.0 or sy != 1.0:
            for d in dets:
                d['box'] *= [sx, sy, sx, sy]
                d['kpts'][:, 0] *= sx
                d['kpts'][:, 1] *= sy
        return dets


class ReidEncoder:
    """Appearance embeddings (unit vectors) for person boxes."""

    def __init__(self, engine_path, max_batch=16):
        self.trt = TrtEngine(engine_path, max_shapes={'crops': (max_batch, 256, 128, 3)})
        self.in_name = self.trt.input_names[0]
        self.out_name = self.trt.output_names[0]
        self.max_batch = max_batch

    def __call__(self, rgb, boxes):
        n = len(boxes)
        if n == 0:
            return np.zeros((0, 512), np.float32)
        ih, iw = rgb.shape[:2]
        feats = []
        for s in range(0, n, self.max_batch):
            chunk = boxes[s:s + self.max_batch]
            view = self.trt.input_view(self.in_name, (len(chunk), 256, 128, 3))
            for k, b in enumerate(chunk):
                x0, y0 = max(int(b[0]), 0), max(int(b[1]), 0)
                x1, y1 = min(int(np.ceil(b[2])), iw), min(int(np.ceil(b[3])), ih)
                if x1 - x0 < 2 or y1 - y0 < 2:
                    view[k] = 0
                    continue
                cv2.resize(rgb[y0:y1, x0:x1], (128, 256), dst=view[k],
                           interpolation=cv2.INTER_LINEAR)
            out = self.trt.infer(shapes={self.in_name: view.shape})[self.out_name]
            feats.append(np.array(out, dtype=np.float32))
        return np.concatenate(feats, axis=0)


def mark_clean(dets, img_w, overlap_frac=0.15, edge_px=4):
    """A detection is 'clean' (safe to learn appearance from) if no other box
    covers more than overlap_frac of it and it is not cut by the left/right
    image edge (a half person is a different appearance)."""
    for i, d in enumerate(dets):
        b = d['box']
        area = max((b[2] - b[0]) * (b[3] - b[1]), 1e-9)
        clean = b[0] > edge_px and b[2] < img_w - edge_px
        for j, o in enumerate(dets):
            if i == j or not clean:
                continue
            ob = o['box']
            iw = min(b[2], ob[2]) - max(b[0], ob[0])
            ih = min(b[3], ob[3]) - max(b[1], ob[1])
            if iw > 0 and ih > 0 and iw * ih / area > overlap_frac:
                clean = False
        d['clean'] = bool(clean)
    return dets
