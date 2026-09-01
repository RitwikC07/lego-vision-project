import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None

        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info('Camera viewer started.')

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to convert color image: {e}'
            )

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to convert depth image: {e}'
            )

    def display_images(self):

        if self.color_image is None:
            return

        cv2.imshow('RGB', self.color_image)

        if self.depth_image is not None:

            # Normalized depth only for visualization.
            depth_visual = cv2.normalize(
                self.depth_image,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )

            depth_visual = depth_visual.astype(np.uint8)

            depth_colormap = cv2.applyColorMap(
                depth_visual,
                cv2.COLORMAP_JET
            )

            cv2.imshow('Depth', depth_colormap)

        key = cv2.waitKey(1)

        if key == ord('q'):
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = CameraViewer()

    try:
        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01
            )

            node.display_images()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()