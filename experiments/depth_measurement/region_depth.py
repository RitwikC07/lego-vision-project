import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthReader(Node):

    def __init__(self):
        super().__init__('depth_reader')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.depth_values = []
        self.max_samples = 100

    def depth_callback(self, msg):

        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        height, width = depth_image.shape

        # Centre pixel
        u = width // 2
        v = height // 2

        # 21 x 21 region
        half_size = 10

        region = depth_image[
            v - half_size:v + half_size + 1,
            u - half_size:u + half_size + 1
        ]

        # Keep only valid depth values
        valid_depth = region[region > 0]

        if valid_depth.size == 0:
            return

        # Use the median depth of the region
        median_depth = np.median(valid_depth)

        self.depth_values.append(median_depth)

        if len(self.depth_values) >= self.max_samples:

            values = np.array(self.depth_values)

            self.get_logger().info(
                f'Collected {len(values)} frames'
            )

            self.get_logger().info(
                f'Region size: 21 x 21'
            )

            self.get_logger().info(
                f'Valid pixels per frame: '
                f'{valid_depth.size}/441'
            )

            self.get_logger().info(
                f'Min median:    {values.min():.2f}'
            )

            self.get_logger().info(
                f'Max median:    {values.max():.2f}'
            )

            self.get_logger().info(
                f'Mean median:   {values.mean():.2f}'
            )

            self.get_logger().info(
                f'Overall median: {np.median(values):.2f}'
            )

            self.get_logger().info(
                f'Std:            {values.std():.2f}'
            )

            self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DepthReader()

    while rclpy.ok() and len(node.depth_values) < node.max_samples:
        rclpy.spin_once(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()