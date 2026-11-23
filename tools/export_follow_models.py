#!/usr/bin/env python3
"""Export the person-following models to ONNX, with preprocessing in the graph."""

import argparse
import os
from pathlib import Path

MODELS = Path.home() / 'labrobot_models'
DOWNLOAD = MODELS / 'download'
ONNX_DIR = MODELS / 'onnx'

POSE_MODELS = ['yolo26n-pose', 'yolo26s-pose', 'yolo11n-pose', 'yolo11s-pose']
REID_MODELS = {'osnet_x0_25_msmt17': 'osnet_x0_25', 'osnet_ain_x1_0_msmt17': 'osnet_ain_x1_0'}


def prepend_image_preprocessing(onnx_path, out_path, h, w):
    """uint8 NHWC 'image' -> float NCHW /255 -> the model's original input."""
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    m = onnx.load(onnx_path)
    g = m.graph
    assert len(g.input) == 1, [i.name for i in g.input]
    orig = g.input[0].name
    new_in = helper.make_tensor_value_info('image', TensorProto.UINT8, [1, h, w, 3])
    scale = numpy_helper.from_array(np.array(1.0 / 255.0, dtype=np.float32), 'pp_scale')
    nodes = [
        helper.make_node('Cast', ['image'], ['pp_float'], to=TensorProto.FLOAT, name='pp_cast'),
        helper.make_node('Mul', ['pp_float', 'pp_scale'], ['pp_scaled'], name='pp_scale_mul'),
        helper.make_node('Transpose', ['pp_scaled'], [orig], perm=[0, 3, 1, 2], name='pp_nchw'),
    ]
    g.initializer.append(scale)
    del g.input[:]
    g.input.append(new_in)
    for n in reversed(nodes):
        g.node.insert(0, n)
    onnx.checker.check_model(m)
    onnx.save(m, out_path)


def export_pose(name):
    from ultralytics import YOLO
    src = DOWNLOAD / f'{name}.pt'
    model = YOLO(str(src))
    raw = model.export(format='onnx', imgsz=[480, 640], opset=17, simplify=True,
                       dynamic=False, batch=1, half=False, device='cpu')
    out = ONNX_DIR / f'{name}_480x640_u8.onnx'
    prepend_image_preprocessing(raw, str(out), 480, 640)
    os.remove(raw)
    print(f'{name}: {out}')
    return out


def export_reid(weights_name, arch):
    import torch
    import torch.nn.functional as F
    from boxmot.reid.core.registry import ReIDModelRegistry

    src = DOWNLOAD / f'{weights_name}.pt'
    n_classes = ReIDModelRegistry.get_nr_classes(src)
    net = ReIDModelRegistry.build_model(arch, weights=src, num_classes=n_classes,
                                        pretrained=False, use_gpu=False)
    report = ReIDModelRegistry.load_deployment_weights(net, src)
    print(f'{weights_name}: weights loaded ({report})')
    net.eval()

    class Wrapped(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m
            self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1) * 255)
            self.register_buffer('std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1) * 255)

        def forward(self, crops):
            x = crops.float().permute(0, 3, 1, 2)
            x = (x - self.mean) / self.std
            f = self.m(x)
            return F.normalize(f, dim=1)

    w = Wrapped(net).eval()
    dummy = torch.randint(0, 255, (2, 256, 128, 3), dtype=torch.uint8)
    out = ONNX_DIR / f'{weights_name}_256x128_u8.onnx'
    with torch.no_grad():
        ref = w(dummy)
        kwargs = dict(input_names=['crops'], output_names=['embedding'], opset_version=17,
                      dynamic_axes={'crops': {0: 'batch'}, 'embedding': {0: 'batch'}})
        try:
            torch.onnx.export(w, (dummy,), str(out), dynamo=False, **kwargs)
        except TypeError:
            torch.onnx.export(w, (dummy,), str(out), **kwargs)
    print(f'{weights_name}: {out}  embedding dim {ref.shape[1]}')
    return out, dummy.numpy(), ref.numpy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', nargs='*', default=None, help=f'any of {POSE_MODELS}')
    ap.add_argument('--reid', nargs='*', default=None, help=f'any of {list(REID_MODELS)}')
    ap.add_argument('--all', action='store_true')
    a = ap.parse_args()
    ONNX_DIR.mkdir(parents=True, exist_ok=True)
    pose = POSE_MODELS if a.all else (a.pose or [])
    reid = list(REID_MODELS) if a.all else (a.reid or [])
    os.chdir(ONNX_DIR)       # ultralytics writes next to the weights / cwd
    for p in pose:
        export_pose(p)
    for r in reid:
        export_reid(r, REID_MODELS[r])


if __name__ == '__main__':
    main()
