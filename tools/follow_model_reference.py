#!/usr/bin/env python3
"""Reference outputs from the ORIGINAL PyTorch models, for parity checks."""

from pathlib import Path

import cv2
import numpy as np

M = Path.home() / 'labrobot_models'


def main():
    import torch
    import torch.nn.functional as F
    import ultralytics
    from boxmot.reid.core.registry import ReIDModelRegistry
    from ultralytics import YOLO

    bus = Path(ultralytics.__file__).parent / 'assets' / 'bus.jpg'
    bgr = cv2.resize(cv2.imread(str(bus)), (640, 480), interpolation=cv2.INTER_AREA)
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    out = {'image': rgb}
    for name in ('yolo26n-pose', 'yolo26s-pose', 'yolo11n-pose', 'yolo11s-pose'):
        r = YOLO(str(M / 'download' / f'{name}.pt'))(bgr, imgsz=[480, 640], conf=0.25,
                                                       verbose=False, device='cpu')[0]
        out[f'{name}_boxes'] = r.boxes.xyxy.numpy()
        out[f'{name}_scores'] = r.boxes.conf.numpy()
        out[f'{name}_kpts'] = r.keypoints.data.numpy()
        print(name, len(r.boxes), 'people', np.round(r.boxes.conf.numpy(), 2))
    boxes = out['yolo26s-pose_boxes']
    crops = np.stack([cv2.resize(rgb[int(b[1]):int(np.ceil(b[3])), int(b[0]):int(np.ceil(b[2]))],
                                 (128, 256), interpolation=cv2.INTER_LINEAR) for b in boxes])
    out['reid_boxes'] = boxes
    mean = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
    for wname, arch in (('osnet_x0_25_msmt17', 'osnet_x0_25'),
                        ('osnet_ain_x1_0_msmt17', 'osnet_ain_x1_0')):
        src = M / 'download' / f'{wname}.pt'
        net = ReIDModelRegistry.build_model(arch, weights=src,
                                            num_classes=ReIDModelRegistry.get_nr_classes(src),
                                            pretrained=False, use_gpu=False)
        ReIDModelRegistry.load_deployment_weights(net, src)
        net.eval()
        with torch.no_grad():
            x = (torch.from_numpy(crops).permute(0, 3, 1, 2).float() / 255 - mean) / std
            out[f'{wname}_emb'] = F.normalize(net(x), dim=1).numpy()
    np.savez(M / 'reference.npz', **out)
    print('saved', M / 'reference.npz')


if __name__ == '__main__':
    main()
