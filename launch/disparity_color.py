#!/usr/bin/env python3
"""Turn /disparity (stereo_msgs/DisparityImage) into a coloured image RViz can show.
Publishes /disparity/color (rgb8): near = red/yellow, far = blue, no data = black."""
import cv2
import numpy as np


def colorize(d, max_disp=128.0):
    valid = d > 0
    norm = np.clip(d / max_disp * 255.0, 0, 255).astype(np.uint8)
    rgb = cv2.applyColorMap(norm, cv2.COLORMAP_JET)[:, :, ::-1].copy()
    rgb[~valid] = 0
    return rgb


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from stereo_msgs.msg import DisparityImage

    class DispColor(Node):
        def __init__(self):
            super().__init__('disparity_color')
            self.pub = self.create_publisher(Image, '/disparity/color', 2)
            self.create_subscription(DisparityImage, '/disparity', self.cb, 2)

        def cb(self, msg):
            im = msg.image
            d = np.frombuffer(im.data, np.float32).reshape(im.height, im.step // 4)[:, :im.width]
            rgb = colorize(d, max(msg.max_disparity, 1.0))
            out = Image()
            out.header = msg.header
            out.height, out.width = rgb.shape[:2]
            out.encoding, out.is_bigendian, out.step = 'rgb8', 0, out.width * 3
            out.data = rgb.tobytes()
            self.pub.publish(out)

    rclpy.init()
    node = DispColor()
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