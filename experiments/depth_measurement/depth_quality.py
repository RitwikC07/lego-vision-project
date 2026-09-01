import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthQuality(Node):

    def __init__(self):
        super().__init__('depth_quality')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.quality_measurements = []
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

        total_pixels = region.size

        # Valid depth values are greater than zero
        valid_depth = region[region > 0]

        valid_pixels = valid_depth.size
        valid_ratio = valid_pixels / total_pixels

        if valid_pixels == 0:
            return

        median_depth = np.median(valid_depth)
        min_depth = np.min(valid_depth)
        max_depth = np.max(valid_depth)
        std_depth = np.std(valid_depth)

        self.quality_measurements.append(
            (
                valid_ratio,
                median_depth,
                min_depth,
                max_depth,
                std_depth
            )
        )

        if len(self.quality_measurements) >= self.max_samples:

            measurements = np.array(self.quality_measurements)

            valid_ratios = measurements[:, 0]
            medians = measurements[:, 1]
            mins = measurements[:, 2]
            maxs = measurements[:, 3]
            stds = measurements[:, 4]

            self.get_logger().info(
                f'Collected {len(measurements)} frames'
            )

            self.get_logger().info(
                '--- Depth Quality ---'
            )

            self.get_logger().info(
                f'Region size:       21 x 21'
            )

            self.get_logger().info(
                f'Valid ratio:       '
                f'{valid_ratios.mean() * 100:.1f}%'
            )

            self.get_logger().info(
                f'Median depth:      '
                f'{np.median(medians):.2f}'
            )

            self.get_logger().info(
                f'Min depth:         '
                f'{np.min(mins):.2f}'
            )

            self.get_logger().info(
                f'Max depth:         '
                f'{np.max(maxs):.2f}'
            )

            self.get_logger().info(
                f'Mean depth std:    '
                f'{stds.mean():.2f}'
            )

            self.get_logger().info(
                f'Median depth std:  '
                f'{np.median(stds):.2f}'
            )

            self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DepthQuality()

    while rclpy.ok() and len(node.quality_measurements) < node.max_samples:
        rclpy.spin_once(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()