#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class DepthEdges(Node):

    def __init__(self):
        super().__init__('depth_edges')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.target_frames = 50
        self.frames_collected = 0

        self.max_edges = []
        self.mean_edges = []
        self.median_edges = []

        self.get_logger().info('Waiting for depth frames...')

    def depth_callback(self, msg):

        if self.frames_collected >= self.target_frames:
            return

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        depth = np.asarray(depth, dtype=np.float32)

        # Horizontal neighbouring-pixel differences
        horizontal_a = depth[:, :-1]
        horizontal_b = depth[:, 1:]

        horizontal_valid = (horizontal_a > 0) & (horizontal_b > 0)

        horizontal_diff = np.abs(
            horizontal_a[horizontal_valid] -
            horizontal_b[horizontal_valid]
        )

        # Vertical neighbouring-pixel differences
        vertical_a = depth[:-1, :]
        vertical_b = depth[1:, :]

        vertical_valid = (vertical_a > 0) & (vertical_b > 0)

        vertical_diff = np.abs(
            vertical_a[vertical_valid] -
            vertical_b[vertical_valid]
        )

        # Combine horizontal and vertical differences
        differences = np.concatenate([
            horizontal_diff,
            vertical_diff
        ])

        if differences.size == 0:
            return

        self.frames_collected += 1

        self.max_edges.append(float(np.max(differences)))
        self.mean_edges.append(float(np.mean(differences)))
        self.median_edges.append(float(np.median(differences)))

        if self.frames_collected % 10 == 0:
            self.get_logger().info(
                f'Collected {self.frames_collected}/{self.target_frames} frames'
            )

        if self.frames_collected == self.target_frames:
            self.report_results()

    def report_results(self):

        self.get_logger().info('--- Depth Edges ---')

        self.get_logger().info(
            f'Max edge:       {np.max(self.max_edges):.2f}'
        )

        self.get_logger().info(
            f'Mean edge:      {np.mean(self.mean_edges):.2f}'
        )

        self.get_logger().info(
            f'Median edge:    {np.median(self.median_edges):.2f}'
        )

        self.get_logger().info(
            f'Edge std:       {np.std(self.median_edges):.2f}'
        )

        self.get_logger().info('Experiment complete.')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = DepthEdges()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()