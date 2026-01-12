import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32

from sensor_msgs_py import point_cloud2
import math


class ToFDistanceNode(Node):
    def __init__(self):
        super().__init__('tof_distance_node')

        # ===============================
        # Parameters
        # ===============================
        self.declare_parameter(
            'points_topic',
            '/link2/tof_west/points'
        )
        self.declare_parameter(
            'distance_topic',
            '/link2/tof_west/distance'
        )
        self.declare_parameter(
            'range_max',
            0.30
        )

        points_topic = self.get_parameter(
            'points_topic'
        ).value
        distance_topic = self.get_parameter(
            'distance_topic'
        ).value
        self.range_max = self.get_parameter(
            'range_max'
        ).value

        self.get_logger().info(
            f"Subscribing to PointCloud: {points_topic}"
        )
        self.get_logger().info(
            f"Publishing distance to: {distance_topic}"
        )

        # ===============================
        # Subscriber / Publisher
        # ===============================
        self.sub = self.create_subscription(
            PointCloud2,
            points_topic,
            self.cb_pointcloud,
            10
        )

        self.pub = self.create_publisher(
            Float32,
            distance_topic,
            10
        )

    # ===============================
    # Callback
    # ===============================
    def cb_pointcloud(self, msg: PointCloud2):
        min_dist = float('inf')
        valid = False

        for x, y, z in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            d = math.sqrt(x * x + y * y + z * z)
            if d < min_dist:
                min_dist = d
                valid = True

        out = Float32()

        if valid:
            out.data = min_dist
        else:
            # No object in FoV → equivalent to "far away"
            out.data = self.range_max

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ToFDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

