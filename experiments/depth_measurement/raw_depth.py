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

        u = width // 2
        v = height // 2

        raw_depth = int(depth_image[v, u])
        
        if raw_depth > 0:
            self.depth_values.append(raw_depth)

        if len(self.depth_values) >= self.max_samples:

            values = np.array(self.depth_values)

            self.get_logger().info(
                f'Collected {len(values)} valid measurements'
            )

            self.get_logger().info(
                f'Min:    {values.min()}'
            )

            self.get_logger().info(
                f'Max:    {values.max()}'
            )

            self.get_logger().info(
                f'Mean:   {values.mean():.2f}'
            )

            self.get_logger().info(
                f'Median: {np.median(values):.2f}'
            )

            self.get_logger().info(
                f'Std:    {values.std():.2f}'
            )

            self.destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DepthReader()

    while rclpy.ok() and node.depth_values.__len__() < node.max_samples:
        rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()