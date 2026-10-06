#!/usr/bin/env python3
"""
Live terrain segmentation (HRNet+OCR, RELLIS-3D weights) on the robot's left camera.

Publishes (display only - Nav2 is not affected):
  /segmentation/labels   mono8, class id per pixel (0..18, see CLASSES in seg_test.py)
  /segmentation/overlay  rgb8, camera image tinted with RELLIS-3D class colours

Run on its own:   python3 ~/SIH/launch/seg_node.py --ros-args -p use_sim_time:=true
or with the pipeline:  ~/SIH/launch/start_pipeline.sh seg:=true
"""
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seg_test import build_model, preprocess, infer, CLASSES  # noqa: E402

# Official RELLIS-3D colours, in the model's class order (RGB)
RELLIS_RGB = np.array([
    [108, 64, 20],    # 0 void/dirt
    [0, 102, 0],      # 1 grass
    [0, 255, 0],      # 2 tree
    [0, 153, 153],    # 3 pole
    [0, 128, 255],    # 4 water
    [0, 0, 255],      # 5 sky
    [255, 255, 0],    # 6 vehicle
    [255, 0, 127],    # 7 object
    [64, 64, 64],     # 8 asphalt
    [255, 0, 0],      # 9 building
    [102, 0, 0],      # 10 log
    [204, 153, 255],  # 11 person
    [102, 0, 204],    # 12 fence
    [255, 153, 204],  # 13 bush
    [170, 170, 170],  # 14 concrete
    [41, 121, 255],   # 15 barrier
    [134, 255, 239],  # 16 puddle
    [99, 66, 34],     # 17 mud
    [110, 22, 138],   # 18 rubble
], np.uint8)


def overlay(rgb, labels, alpha=0.55):
    return (rgb * (1 - alpha) + RELLIS_RGB[labels] * alpha).astype(np.uint8)


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class SegNode(Node):
        def __init__(self):
            super().__init__('segmentation')
            P = self.declare_parameter
            self.rate = P('rate', 3.0).value                  # inferences per second
            self.fp16 = P('fp16', True).value
            self.w, self.h = P('width', 640).value, P('height', 480).value
            topic = P('image', '/stereo/left/image_raw').value
            self.dev = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.model, _ = build_model(self.dev)
            self.get_logger().info(f'HRNet+OCR loaded on {self.dev}, {self.rate} Hz, fp16={self.fp16}')
            self.pub_lab = self.create_publisher(Image, '/segmentation/labels', 2)
            self.pub_ov = self.create_publisher(Image, '/segmentation/overlay', 2)
            self.latest, self.done_stamp = None, None
            self.create_subscription(Image, topic, lambda m: setattr(self, 'latest', m), 2)
            self.create_timer(1.0 / self.rate, self.tick)
            self.times = []

        def tick(self):
            msg = self.latest
            if msg is None or msg.header.stamp == self.done_stamp:
                return
            self.done_stamp = msg.header.stamp
            t = time.perf_counter()
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            rgb = a.reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                rgb = rgb[:, :, ::-1]
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            x, img_bgr = preprocess(bgr, (self.w, self.h), self.dev)
            labels = infer(self.model, x, self.fp16).to(torch.uint8).cpu().numpy()
            ov = overlay(np.ascontiguousarray(img_bgr[:, :, ::-1]), labels)

            for pub, data, enc, step in ((self.pub_lab, labels, 'mono8', self.w),
                                         (self.pub_ov, ov, 'rgb8', self.w * 3)):
                m = Image()
                m.header = msg.header
                m.height, m.width = self.h, self.w
                m.encoding, m.is_bigendian, m.step = enc, 0, step
                m.data = data.tobytes()
                pub.publish(m)

            self.times.append(time.perf_counter() - t)
            if len(self.times) == 15:
                counts = np.bincount(labels.ravel(), minlength=len(CLASSES)) / labels.size
                top = ', '.join(f'{CLASSES[i]} {100 * counts[i]:.0f}%' for i in np.argsort(-counts)[:3])
                self.get_logger().info(f'{1000 * sum(self.times) / 15:.0f} ms/frame | top classes: {top}')
                self.times = []

    rclpy.init()
    node = SegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
