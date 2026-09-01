#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthPlane(Node):

    def __init__(self):
        super().__init__('depth_plane')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.frame_count = 0
        self.target_frames = 100

        self.depth_values = []

        self.image_width = None
        self.image_height = None

        self.get_logger().info('Waiting for depth frames...')

    def depth_callback(self, msg):

        if self.frame_count >= self.target_frames:
            return

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        depth = np.asarray(depth)

        if self.image_width is None:
            self.image_height, self.image_width = depth.shape

            self.get_logger().info(
                f'Image size: {self.image_width} x {self.image_height}'
            )

        # ---------------------------------------------------------
        # TABLE-ONLY REGION
        #
        # We deliberately avoid the extreme image boundaries.
        # This region should contain ONLY the table.
        # ---------------------------------------------------------

        h, w = depth.shape

        x1 = int(w * 0.25)
        x2 = int(w * 0.75)

        y1 = int(h * 0.25)
        y2 = int(h * 0.75)

        region = depth[y1:y2, x1:x2]

        # Valid depth pixels
        valid_mask = (
            np.isfinite(region) &
            (region > 0)
        )

        if np.count_nonzero(valid_mask) < 100:
            return

        # Store valid pixels together with their image coordinates
        ys, xs = np.where(valid_mask)

        z = region[valid_mask].astype(np.float64)

        # Convert region coordinates to full-image coordinates
        xs_full = xs + x1
        ys_full = ys + y1

        self.depth_values.append(
            (
                xs_full.astype(np.float64),
                ys_full.astype(np.float64),
                z
            )
        )

        self.frame_count += 1

        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'Collected {self.frame_count}/{self.target_frames} frames'
            )

        if self.frame_count == self.target_frames:
            self.analyze()

    def analyze(self):

        # Combine all frames
        x = np.concatenate([v[0] for v in self.depth_values])
        y = np.concatenate([v[1] for v in self.depth_values])
        z = np.concatenate([v[2] for v in self.depth_values])

        # ---------------------------------------------------------
        # BASIC DEPTH STATISTICS
        # ---------------------------------------------------------

        median_depth = np.median(z)
        mean_depth = np.mean(z)
        min_depth = np.min(z)
        max_depth = np.max(z)
        std_depth = np.std(z)

        # ---------------------------------------------------------
        # FIT DEPTH PLANE
        #
        # Z = ax + by + c
        # ---------------------------------------------------------

        A = np.column_stack((x, y, np.ones_like(x)))

        coefficients, _, _, _ = np.linalg.lstsq(
            A,
            z,
            rcond=None
        )

        a, b, c = coefficients

        # Predicted depth from fitted plane
        z_plane = A @ coefficients

        residuals = z - z_plane

        residual_mean = np.mean(residuals)
        residual_std = np.std(residuals)

        residual_min = np.min(residuals)
        residual_max = np.max(residuals)

        # ---------------------------------------------------------
        # PLANE DEPTH AT IMAGE CENTRE
        # ---------------------------------------------------------

        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        centre_depth = (
            a * cx +
            b * cy +
            c
        )

        # ---------------------------------------------------------
        # DEPTH CHANGE ACROSS IMAGE
        # ---------------------------------------------------------

        left_depth = a * 0 + b * cy + c
        right_depth = a * (self.image_width - 1) + b * cy + c

        top_depth = a * cx + b * 0 + c
        bottom_depth = (
            a * cx +
            b * (self.image_height - 1) +
            c
        )

        horizontal_change = right_depth - left_depth
        vertical_change = bottom_depth - top_depth

        # ---------------------------------------------------------
        # PRINT RESULTS
        # ---------------------------------------------------------

        self.get_logger().info('--- Depth Plane ---')

        self.get_logger().info(
            f'Total valid samples:    {len(z)}'
        )

        self.get_logger().info(
            f'Mean depth:             {mean_depth:.2f}'
        )

        self.get_logger().info(
            f'Median depth:           {median_depth:.2f}'
        )

        self.get_logger().info(
            f'Min depth:              {min_depth:.2f}'
        )

        self.get_logger().info(
            f'Max depth:              {max_depth:.2f}'
        )

        self.get_logger().info(
            f'Depth std:              {std_depth:.2f}'
        )

        self.get_logger().info(
            f'--- Fitted plane ---'
        )

        self.get_logger().info(
            f'a:                      {a:.8f}'
        )

        self.get_logger().info(
            f'b:                      {b:.8f}'
        )

        self.get_logger().info(
            f'c:                      {c:.2f}'
        )

        self.get_logger().info(
            f'Centre plane depth:     {centre_depth:.2f}'
        )

        self.get_logger().info(
            f'Horizontal change:      {horizontal_change:.2f}'
        )

        self.get_logger().info(
            f'Vertical change:        {vertical_change:.2f}'
        )

        self.get_logger().info(
            f'--- Plane residuals ---'
        )

        self.get_logger().info(
            f'Residual mean:          {residual_mean:.2f}'
        )

        self.get_logger().info(
            f'Residual std:           {residual_std:.2f}'
        )

        self.get_logger().info(
            f'Residual min:           {residual_min:.2f}'
        )

        self.get_logger().info(
            f'Residual max:           {residual_max:.2f}'
        )

        self.get_logger().info(
            'Experiment complete.'
        )

        # Stop receiving frames
        self.destroy_subscription(self.subscription)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DepthPlane()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()