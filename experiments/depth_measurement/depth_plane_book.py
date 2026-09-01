#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthPlaneBookExperiment(Node):

    def __init__(self):
        super().__init__('depth_plane_book_experiment')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # ---------------------------------------------------------
        # Experiment parameters
        # ---------------------------------------------------------

        self.num_frames = 100

        # Central analysis region.
        # These are FULL IMAGE coordinates.
        self.x_start_fraction = 0.25
        self.x_end_fraction = 0.75
        self.y_start_fraction = 0.25
        self.y_end_fraction = 0.75

        # Object is closer to the camera than the table,
        # therefore residual should be negative.
        self.object_threshold_mm = -20.0

        # Known table plane from depth_plane.py:
        #
        # Z_table(x,y) = a*x + b*y + c
        #
        self.a = -0.01136354
        self.b = -0.00771668
        self.c = 424.66

        # ---------------------------------------------------------
        # Frame collection
        # ---------------------------------------------------------

        self.frames = []

        self.image_width = None
        self.image_height = None

        self.finished = False

        self.get_logger().info(
            'Waiting for depth frames...'
        )

    # -------------------------------------------------------------
    # Depth callback
    # -------------------------------------------------------------

    def depth_callback(self, msg):

        if self.finished:
            return

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        depth = np.asarray(depth)

        if self.image_width is None:
            self.image_height, self.image_width = depth.shape

            self.get_logger().info(
                f'Depth image: '
                f'{self.image_width} x {self.image_height}'
            )

        self.frames.append(depth.astype(np.float32))

        if len(self.frames) >= self.num_frames:

            self.finished = True

            self.run_experiment()

            # Stop receiving frames.
            self.destroy_subscription(self.subscription)

    # -------------------------------------------------------------
    # Main experiment
    # -------------------------------------------------------------

    def run_experiment(self):

        print()
        print('=' * 70)
        print('DEPTH PLANE + BOOK LOCALIZATION EXPERIMENT')
        print('=' * 70)

        print()
        print(f'Image size: {self.image_width} x {self.image_height}')
        print(f'Frames collected: {len(self.frames)}')

        # ---------------------------------------------------------
        # Convert invalid zero depth values to NaN
        # ---------------------------------------------------------

        stack = np.stack(self.frames, axis=0)

        stack[stack == 0] = np.nan

        # ---------------------------------------------------------
        # Temporal median fusion
        #
        # For every pixel:
        #
        # Z(x,y) = median over 100 frames
        #
        # NaN values are ignored.
        # ---------------------------------------------------------

        depth_median = np.nanmedian(stack, axis=0)

        # ---------------------------------------------------------
        # Validity
        # ---------------------------------------------------------

        valid_mask = np.isfinite(depth_median)

        valid_pixels = np.count_nonzero(valid_mask)
        total_pixels = depth_median.size

        valid_ratio = (
            100.0 * valid_pixels / total_pixels
        )

        print()
        print('Depth validity:')
        print(f'  valid pixels: {valid_pixels} / {total_pixels}')
        print(f'  valid ratio: {valid_ratio:.2f}%')

        # ---------------------------------------------------------
        # Define analysis region in FULL IMAGE coordinates
        # ---------------------------------------------------------

        x0 = int(self.image_width * self.x_start_fraction)
        x1 = int(self.image_width * self.x_end_fraction)

        y0 = int(self.image_height * self.y_start_fraction)
        y1 = int(self.image_height * self.y_end_fraction)

        print()
        print('Analysis region:')
        print(f'  x = {x0}:{x1}')
        print(f'  y = {y0}:{y1}')
        print(f'  width = {x1 - x0} px')
        print(f'  height = {y1 - y0} px')

        # ---------------------------------------------------------
        # Extract analysis region
        # ---------------------------------------------------------

        depth_region = depth_median[y0:y1, x0:x1]
        valid_region = valid_mask[y0:y1, x0:x1]

        # ---------------------------------------------------------
        # Construct FULL IMAGE coordinate grid
        #
        # This is important.
        #
        # x and y represent actual camera-image coordinates,
        # not coordinates relative to the crop.
        # ---------------------------------------------------------

        ys, xs = np.indices(
            depth_median.shape,
            dtype=np.float32
        )

        # ---------------------------------------------------------
        # Mathematical table plane
        #
        # Z_table(x,y) = a*x + b*y + c
        # ---------------------------------------------------------

        table_plane = (
            self.a * xs
            + self.b * ys
            + self.c
        )

        # ---------------------------------------------------------
        # Residual
        #
        # residual = measured depth - expected table depth
        #
        # Table:
        #     residual ≈ 0
        #
        # Object above table:
        #     measured depth < table depth
        #     residual < 0
        # ---------------------------------------------------------

        residual = depth_median - table_plane

        residual_region = residual[y0:y1, x0:x1]

        # ---------------------------------------------------------
        # Centre measurement
        # ---------------------------------------------------------

        centre_x = self.image_width // 2
        centre_y = self.image_height // 2

        centre_depth = depth_median[
            centre_y,
            centre_x
        ]

        centre_plane = table_plane[
            centre_y,
            centre_x
        ]

        centre_residual = (
            centre_depth - centre_plane
            if np.isfinite(centre_depth)
            else np.nan
        )

        print()
        print('Centre measurement:')
        print(f'  pixel: ({centre_x}, {centre_y})')
        print(f'  table plane: {centre_plane:.2f} mm')
        print(f'  measured depth: {centre_depth:.2f} mm')
        print(f'  residual: {centre_residual:.2f} mm')

        # ---------------------------------------------------------
        # Residual statistics
        # ---------------------------------------------------------

        valid_residual = residual_region[
            np.isfinite(residual_region)
        ]

        print()
        print('Residual statistics:')
        print(f'  median: {np.median(valid_residual):.2f} mm')
        print(f'  mean: {np.mean(valid_residual):.2f} mm')
        print(f'  std: {np.std(valid_residual):.2f} mm')
        print(f'  min: {np.min(valid_residual):.2f} mm')
        print(f'  max: {np.max(valid_residual):.2f} mm')

        # ---------------------------------------------------------
        # Object detection
        # ---------------------------------------------------------

        object_mask = (
            np.isfinite(residual_region)
            &
            (residual_region < self.object_threshold_mm)
        )

        object_pixels = np.count_nonzero(object_mask)

        region_pixels = object_mask.size

        object_ratio = (
            100.0 * object_pixels / region_pixels
        )

        print()
        print('Object detection:')
        print(
            f'  threshold: residual < '
            f'{self.object_threshold_mm:.1f} mm'
        )
        print(f'  object pixels: {object_pixels}')
        print(f'  object ratio: {object_ratio:.2f}%')

        # ---------------------------------------------------------
        # Object localization
        #
        # np.where() returns coordinates RELATIVE TO THE REGION.
        #
        # Convert them back to FULL IMAGE coordinates by adding
        # x0 and y0.
        # ---------------------------------------------------------

        object_y_local, object_x_local = np.where(
            object_mask
        )

        if object_pixels == 0:

            print()
            print('Object localization:')
            print('  NO OBJECT DETECTED.')

            return

        object_x = object_x_local + x0
        object_y = object_y_local + y0

        # ---------------------------------------------------------
        # Bounding box
        # ---------------------------------------------------------

        x_min = int(np.min(object_x))
        x_max = int(np.max(object_x))

        y_min = int(np.min(object_y))
        y_max = int(np.max(object_y))

        width = x_max - x_min + 1
        height = y_max - y_min + 1

        # ---------------------------------------------------------
        # Geometric bounding-box centre
        # ---------------------------------------------------------

        bbox_centre_x = (
            x_min + x_max
        ) / 2.0

        bbox_centre_y = (
            y_min + y_max
        ) / 2.0

        # ---------------------------------------------------------
        # Pixel centroid
        #
        # This is the arithmetic mean of all detected pixel
        # coordinates.
        # ---------------------------------------------------------

        centroid_x = float(np.mean(object_x))
        centroid_y = float(np.mean(object_y))

        # ---------------------------------------------------------
        # Print localization results
        # ---------------------------------------------------------

        print()
        print('Object localization:')
        print(
            f'  bounding box: '
            f'x={x_min}:{x_max}, '
            f'y={y_min}:{y_max}'
        )

        print(
            f'  bounding-box size: '
            f'{width} x {height} px'
        )

        print(
            f'  bounding-box centre: '
            f'({bbox_centre_x:.2f}, '
            f'{bbox_centre_y:.2f}) px'
        )

        print(
            f'  pixel centroid: '
            f'({centroid_x:.2f}, '
            f'{centroid_y:.2f}) px'
        )

        print(
            f'  detected pixels: '
            f'{object_pixels}'
        )

        # ---------------------------------------------------------
        # Object residual statistics
        # ---------------------------------------------------------

        object_residuals = residual[
            object_y,
            object_x
        ]

        print()
        print('Detected-object residuals:')
        print(
            f'  median: '
            f'{np.median(object_residuals):.2f} mm'
        )

        print(
            f'  mean: '
            f'{np.mean(object_residuals):.2f} mm'
        )

        print(
            f'  std: '
            f'{np.std(object_residuals):.2f} mm'
        )

        print(
            f'  min: '
            f'{np.min(object_residuals):.2f} mm'
        )

        print(
            f'  max: '
            f'{np.max(object_residuals):.2f} mm'
        )

        print()
        print('=' * 70)
        print('EXPERIMENT COMPLETE')
        print('=' * 70)
        print()


def main(args=None):

    rclpy.init(args=args)

    node = DepthPlaneBookExperiment()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()