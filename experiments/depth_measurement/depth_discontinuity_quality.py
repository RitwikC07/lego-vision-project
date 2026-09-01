#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthDiscontinuityQuality(Node):

    def __init__(self):
        super().__init__('depth_discontinuity_quality')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.frame_count = 0
        self.target_frames = 100

        self.centre_depths = []
        self.outer_depths = []

        self.centre_valid_ratios = []
        self.outer_valid_ratios = []

        self.finished = False

    def depth_callback(self, msg):

        if self.finished:
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        height, width = depth_image.shape

        # --------------------------------------------------
        # Centre region: 21 x 21 pixels
        # --------------------------------------------------

        cx = width // 2
        cy = height // 2

        half_size = 10

        centre = depth_image[
            cy - half_size:cy + half_size + 1,
            cx - half_size:cx + half_size + 1
        ]

        # --------------------------------------------------
        # Outer region
        #
        # Same idea as depth_discontinuity.py:
        # take a larger region around the centre and
        # exclude the centre region.
        # --------------------------------------------------

        outer_half_size = 30

        outer = depth_image[
            cy - outer_half_size:cy + outer_half_size + 1,
            cx - outer_half_size:cx + outer_half_size + 1
        ]

        # Mask out the centre region from the outer region
        outer_mask = np.ones(outer.shape, dtype=bool)

        outer_centre_start = outer_half_size - half_size
        outer_centre_end = outer_half_size + half_size + 1

        outer_mask[
            outer_centre_start:outer_centre_end,
            outer_centre_start:outer_centre_end
        ] = False

        outer = outer[outer_mask]

        # --------------------------------------------------
        # Valid depth pixels
        #
        # RealSense depth image is 16UC1.
        # A value of zero means invalid / no depth.
        # --------------------------------------------------

        centre_valid = centre[centre > 0]
        outer_valid = outer[outer > 0]

        if len(centre_valid) == 0 or len(outer_valid) == 0:
            return

        # --------------------------------------------------
        # Measurements for this frame
        # --------------------------------------------------

        centre_median = float(np.median(centre_valid))
        outer_median = float(np.median(outer_valid))

        centre_valid_ratio = (
            len(centre_valid) / centre.size
        )

        outer_valid_ratio = (
            len(outer_valid) / outer.size
        )

        self.centre_depths.append(centre_median)
        self.outer_depths.append(outer_median)

        self.centre_valid_ratios.append(centre_valid_ratio)
        self.outer_valid_ratios.append(outer_valid_ratio)

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'Collected {self.frame_count}/{self.target_frames} frames'
            )

        if self.frame_count >= self.target_frames:
            self.finished = True
            self.print_results()

    def print_results(self):

        centre_depths = np.array(self.centre_depths)
        outer_depths = np.array(self.outer_depths)

        centre_valid = np.array(self.centre_valid_ratios)
        outer_valid = np.array(self.outer_valid_ratios)

        depth_difference = centre_depths - outer_depths

        self.get_logger().info('--- Depth Discontinuity + Quality ---')

        self.get_logger().info(
            f'Centre median depth:       '
            f'{np.median(centre_depths):.2f}'
        )

        self.get_logger().info(
            f'Centre valid ratio:        '
            f'{np.mean(centre_valid) * 100:.1f}%'
        )

        self.get_logger().info(
            f'Outer median depth:        '
            f'{np.median(outer_depths):.2f}'
        )

        self.get_logger().info(
            f'Outer valid ratio:         '
            f'{np.mean(outer_valid) * 100:.1f}%'
        )

        self.get_logger().info(
            f'Depth difference:          '
            f'{np.median(depth_difference):.2f}'
        )

        self.get_logger().info(
            f'Difference std:            '
            f'{np.std(depth_difference):.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = DepthDiscontinuityQuality()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()