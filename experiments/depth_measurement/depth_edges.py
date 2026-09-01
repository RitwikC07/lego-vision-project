import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class DepthEdgesQuality(Node):

    def __init__(self):
        super().__init__('depth_edges_quality')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.frames_required = 50
        self.frames = []

        self.threshold = 20.0

        self.get_logger().info("Waiting for depth frames...")

    def depth_callback(self, msg):

        if len(self.frames) >= self.frames_required:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

            depth = np.asarray(depth, dtype=np.float32)

            # Store frame
            self.frames.append(depth.copy())

            count = len(self.frames)

            if count % 10 == 0:
                self.get_logger().info(
                    f"Collected {count}/{self.frames_required} frames"
                )

            if count == self.frames_required:
                self.analyze()

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def analyze(self):

        frames = np.stack(self.frames)

        num_frames, height, width = frames.shape

        # ---------------------------------------------------------
        # 1. Valid depth
        # ---------------------------------------------------------

        valid = frames > 0

        valid_ratio = np.mean(valid)

        # ---------------------------------------------------------
        # 2. Calculate depth edges for every frame
        #
        # Compare neighbouring pixels horizontally and vertically.
        # Only compare pixels where both depths are valid.
        # ---------------------------------------------------------

        edge_maps = np.zeros(
            (num_frames, height, width),
            dtype=bool
        )

        edge_counts = []
        valid_counts = []

        for i in range(num_frames):

            depth = frames[i]
            valid_frame = valid[i]

            valid_counts.append(np.sum(valid_frame))

            # Horizontal difference
            horizontal_valid = (
                valid_frame[:, 1:] &
                valid_frame[:, :-1]
            )

            horizontal_diff = np.abs(
                depth[:, 1:] - depth[:, :-1]
            )

            horizontal_edges = (
                horizontal_valid &
                (horizontal_diff >= self.threshold)
            )

            # Put horizontal edges into the right pixels
            edge_maps[i, :, 1:] |= horizontal_edges

            # Vertical difference
            vertical_valid = (
                valid_frame[1:, :] &
                valid_frame[:-1, :]
            )

            vertical_diff = np.abs(
                depth[1:, :] - depth[:-1, :]
            )

            vertical_edges = (
                vertical_valid &
                (vertical_diff >= self.threshold)
            )

            edge_maps[i, 1:, :] |= vertical_edges

            edge_counts.append(np.sum(edge_maps[i]))

        # ---------------------------------------------------------
        # 3. Persistence
        #
        # IMPORTANT:
        # Each pixel can contribute at most once per frame.
        # Therefore persistence can never exceed 50/50.
        # ---------------------------------------------------------

        persistence = np.sum(edge_maps, axis=0)

        max_persistence = int(np.max(persistence))

        persistent_50 = np.sum(
            persistence >= int(0.50 * num_frames)
        )

        persistent_80 = np.sum(
            persistence >= int(0.80 * num_frames)
        )

        persistent_100 = np.sum(
            persistence == num_frames
        )

        # ---------------------------------------------------------
        # 4. Most persistent pixel
        # ---------------------------------------------------------

        max_positions = np.argwhere(
            persistence == max_persistence
        )

        if len(max_positions) > 0:

            y, x = max_positions[0]

            most_persistent = f"({x}, {y})"

        else:

            most_persistent = "N/A"

        # ---------------------------------------------------------
        # 5. Strongest edge magnitude
        # ---------------------------------------------------------

        max_edge = 0.0

        for i in range(num_frames):

            depth = frames[i]

            valid_frame = valid[i]

            # Horizontal
            h_valid = (
                valid_frame[:, 1:] &
                valid_frame[:, :-1]
            )

            h_diff = np.abs(
                depth[:, 1:] - depth[:, :-1]
            )

            if np.any(h_valid):

                max_edge = max(
                    max_edge,
                    float(np.max(h_diff[h_valid]))
                )

            # Vertical
            v_valid = (
                valid_frame[1:, :] &
                valid_frame[:-1, :]
            )

            v_diff = np.abs(
                depth[1:, :] - depth[:-1, :]
            )

            if np.any(v_valid):

                max_edge = max(
                    max_edge,
                    float(np.max(v_diff[v_valid]))
                )

        # ---------------------------------------------------------
        # 6. Statistics
        # ---------------------------------------------------------

        avg_edge_count = float(
            np.mean(edge_counts)
        )

        edge_ratio = (
            avg_edge_count / (width * height)
        ) * 100.0

        # ---------------------------------------------------------
        # 7. Report
        # ---------------------------------------------------------

        self.get_logger().info(
            "--- Depth Edges + Quality ---"
        )

        self.get_logger().info(
            f"Image size:             {width} x {height}"
        )

        self.get_logger().info(
            f"Threshold:              {self.threshold:.1f}"
        )

        self.get_logger().info(
            f"Valid depth ratio:      {valid_ratio * 100:.1f}%"
        )

        self.get_logger().info(
            f"Average edge count:     {avg_edge_count:.1f}"
        )

        self.get_logger().info(
            f"Edge ratio:             {edge_ratio:.4f}%"
        )

        self.get_logger().info(
            f"Max edge:               {max_edge:.2f}"
        )

        self.get_logger().info(
            f"Persistent >= 50%:      {persistent_50}"
        )

        self.get_logger().info(
            f"Persistent >= 80%:      {persistent_80}"
        )

        self.get_logger().info(
            f"Persistent = 100%:      {persistent_100}"
        )

        self.get_logger().info(
            f"Max persistence:        "
            f"{max_persistence}/{num_frames}"
        )

        self.get_logger().info(
            f"Most persistent pixel:  {most_persistent}"
        )

        self.get_logger().info(
            "Experiment complete."
        )

        # Stop receiving frames
        self.destroy_subscription(self.subscription)


def main(args=None):

    rclpy.init(args=args)

    node = DepthEdgesQuality()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()