import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthDiscontinuity(Node):

    def __init__(self):
        super().__init__('depth_discontinuity')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.measurements = []
        self.max_samples = 100
        self.finished = False

    def depth_callback(self, msg):

        if self.finished:
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        height, width = depth_image.shape

        u = width // 2
        v = height // 2

        # 21 x 21 region
        half_size = 10

        region = depth_image[
            v - half_size:v + half_size + 1,
            u - half_size:u + half_size + 1
        ]

        # Centre pixel
        centre_depth = depth_image[v, u]

        if centre_depth == 0:
            return

        # Remove the central 5 x 5 area.
        # Everything remaining is the outer ring.
        outer_region = region.copy()

        outer_region[
            half_size - 2:half_size + 3,
            half_size - 2:half_size + 3
        ] = 0

        outer_valid = outer_region[outer_region > 0]

        if outer_valid.size == 0:
            return

        outer_median = np.median(outer_valid)

        depth_difference = outer_median - centre_depth

        self.measurements.append(
            (
                centre_depth,
                outer_median,
                depth_difference
            )
        )

        # Progress information
        if len(self.measurements) % 20 == 0:
            self.get_logger().info(
                f'Collected {len(self.measurements)}/'
                f'{self.max_samples} frames'
            )

        if len(self.measurements) >= self.max_samples:

            measurements = np.array(self.measurements)

            centre = measurements[:, 0]
            outer = measurements[:, 1]
            difference = measurements[:, 2]

            self.get_logger().info(
                '--- Depth Discontinuity ---'
            )

            self.get_logger().info(
                f'Centre median:       '
                f'{np.median(centre):.2f}'
            )

            self.get_logger().info(
                f'Outer median:        '
                f'{np.median(outer):.2f}'
            )

            self.get_logger().info(
                f'Depth difference:    '
                f'{np.median(difference):.2f}'
            )

            self.get_logger().info(
                f'Difference std:      '
                f'{np.std(difference):.2f}'
            )

            self.finished = True


def main(args=None):

    rclpy.init(args=args)

    node = DepthDiscontinuity()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()