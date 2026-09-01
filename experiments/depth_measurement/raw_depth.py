import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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

        self.measured = False

    def depth_callback(self, msg):

        if self.measured:
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        height, width = depth_image.shape

        u = width // 2
        v = height // 2

        raw_depth = depth_image[v, u]

        self.get_logger().info(
            f'Image size: {width} x {height}'
        )

        self.get_logger().info(
            f'Pixel: ({u}, {v})'
        )

        self.get_logger().info(
            f'Raw depth value: {raw_depth}'
        )

        self.measured = True


def main(args=None):

    rclpy.init(args=args)

    node = DepthReader()

    while rclpy.ok() and not node.measured:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()