#!/usr/bin/env python3
"""
Standalone test of the HRNet+OCR terrain model (no ROS).

  python3 ~/SIH/launch/seg_test.py                          # default test image
  python3 ~/SIH/launch/seg_test.py --image some.png

Checks that best.pth loads correctly, measures speed on the GPU at 640x480 (fp32 and fp16),
and saves a colour class map + overlay to ~/SIH/ML_Models/seg_test_out/.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as F

ML = os.path.expanduser('~/SIH/ML_Models')
REPO = os.path.join(ML, 'rellis_repo/benchmarks/HRNet-Semantic-Segmentation-HRNet-OCR')
CFG = os.path.join(REPO, 'experiments/rellis/seg_hrnet_ocr_w48_train_512x1024_sgd_lr1e-3_wd5e-4_bs_12_epoch484.yaml')
WEIGHTS = os.path.join(ML, 'HRNet+OCR/best.pth')
OUT = os.path.join(ML, 'seg_test_out')

MEAN = np.array([0.54218053, 0.64250553, 0.56620195], np.float32)    # RELLIS-3D (from rellis.py)
STD = np.array([0.54218052, 0.64250552, 0.56620194], np.float32)

CLASSES = ['void/dirt', 'grass', 'tree', 'pole', 'water', 'sky', 'vehicle', 'object', 'asphalt',
           'building', 'log', 'person', 'fence', 'bush', 'concrete', 'barrier', 'puddle', 'mud', 'rubble']
PALETTE = np.array([  # RGB
    [120, 100, 80], [60, 180, 60], [20, 100, 20], [200, 200, 0], [30, 90, 230], [150, 200, 255],
    [255, 0, 255], [255, 120, 0], [90, 90, 90], [160, 60, 60], [140, 80, 20], [255, 0, 0],
    [200, 150, 100], [100, 160, 40], [180, 180, 180], [255, 200, 0], [0, 200, 200], [110, 70, 30],
    [170, 120, 170]], np.uint8)


def build_model(device, weights=WEIGHTS, cfg_file=CFG, repo=REPO):
    if not hasattr(np, 'int'):
        np.int = int    # the RELLIS HRNet code uses np.int, removed in numpy >= 1.24
    sys.path.insert(0, os.path.join(repo, 'lib'))
    from config import config
    import models.seg_hrnet_ocr as seg
    config.defrost()
    config.merge_from_file(cfg_file)
    config.MODEL.PRETRAINED = ''           # never look for the ImageNet backbone file
    config.freeze()
    model = seg.get_seg_model(config)

    ck = torch.load(weights, map_location='cpu', weights_only=False)
    sd = ck.get('state_dict', ck) if isinstance(ck, dict) else ck
    def clean(k):                     # strip training wrappers: 'module.' (multi-GPU) and 'model.' (FullModel)
        while k.startswith(('module.', 'model.')):
            k = k.split('.', 1)[1]
        return k
    sd = {clean(k): v for k, v in sd.items()}
    own = model.state_dict()
    used = {k: v for k, v in sd.items() if k in own and v.shape == own[k].shape}
    missing = [k for k in own if k not in used]
    print(f'weights: {len(used)}/{len(own)} model tensors loaded, {len(missing)} missing, '
          f'{len([k for k in sd if k not in own])} unused in file')
    if len(used) < 0.95 * len(own):
        print('  first missing keys:', missing[:5])
        sys.exit('ERROR - weights do not match this model code (wrong repo/config?)')
    own.update(used)
    model.load_state_dict(own)
    return model.to(device).eval(), config


def preprocess(bgr, size, device):
    img = cv2.resize(bgr, size, interpolation=cv2.INTER_LINEAR)
    x = (img[:, :, ::-1].astype(np.float32) / 255.0 - MEAN) / STD
    return torch.from_numpy(x.transpose(2, 0, 1).copy())[None].to(device), img


@torch.no_grad()
def infer(model, x, fp16=False):
    with torch.autocast('cuda', dtype=torch.float16, enabled=fp16 and x.is_cuda):
        out = model(x)
    logits = out[-1] if isinstance(out, (list, tuple)) else out       # main OCR head
    logits = F.interpolate(logits.float(), size=x.shape[2:], mode='bilinear', align_corners=False)
    return logits.argmax(1)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--image', default=os.path.expanduser('~/SIH/perception/test_frames/color_0002.png'))
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--runs', type=int, default=10)
    a = ap.parse_args()

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    print('device:', dev, torch.cuda.get_device_name(0) if dev == 'cuda' else '')
    model, _ = build_model(dev)
    bgr = cv2.imread(a.image)
    if bgr is None:
        sys.exit('cannot read image ' + a.image)
    x, img = preprocess(bgr, (a.width, a.height), dev)

    for fp16 in (False, True):
        for _ in range(3):
            infer(model, x, fp16)                                     # warm-up
        if dev == 'cuda':
            torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats()
        t = time.perf_counter()
        for _ in range(a.runs):
            pred = infer(model, x, fp16)
        if dev == 'cuda':
            torch.cuda.synchronize()
        ms = 1000 * (time.perf_counter() - t) / a.runs
        mem = torch.cuda.max_memory_allocated() / 2**20 if dev == 'cuda' else 0
        print(f'{"fp16" if fp16 else "fp32"}: {ms:.0f} ms/frame at {a.width}x{a.height} '
              f'(~{1000 / ms:.1f} fps), peak GPU memory {mem:.0f} MB')

    p = pred.cpu().numpy()
    counts = np.bincount(p.ravel(), minlength=len(CLASSES)) / p.size
    print('classes in this image:')
    for i in np.argsort(-counts):
        if counts[i] > 0.005:
            print(f'  {CLASSES[i]:10s} {100 * counts[i]:5.1f}%')
    os.makedirs(OUT, exist_ok=True)
    color = PALETTE[p][:, :, ::-1]
    base = os.path.splitext(os.path.basename(a.image))[0]
    cv2.imwrite(os.path.join(OUT, base + '_classes.png'), color)
    cv2.imwrite(os.path.join(OUT, base + '_overlay.png'), cv2.addWeighted(img, 0.5, color, 0.5, 0))
    print('saved to', OUT)


if __name__ == '__main__':
    main()