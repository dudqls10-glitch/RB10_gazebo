import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math


class ToFFoVMarker(Node):
    def __init__(self):
        super().__init__('tof_fov_marker')

        self.pub = self.create_publisher(
            Marker,
            '/link2/tof_west/fov_marker',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_marker)

        # ===== FoV parameters =====
        self.frame_id = 'link2'          # RViz Fixed Frame은 world
        self.range_max = 0.30            # 30 cm
        self.fov = math.radians(150.0)   # 150 deg
        self.segments = 40               # 부채꼴 해상도

        # ===== 센서 위치 (link2 기준) =====
        self.sensor_x = 0.0
        self.sensor_y = -0.05            # west 방향
        self.sensor_z = 0.0

        # ===== 센서 방향 (yaw 회전) =====
        # west = -90 deg ( +X 기준 )
        self.yaw = -math.pi / 2.0

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'tof_fov'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        # Pose = 센서 위치
        marker.pose.position.x = self.sensor_x
        marker.pose.position.y = self.sensor_y
        marker.pose.position.z = self.sensor_z

        # Orientation (yaw)
        marker.pose.orientation.z = math.sin(self.yaw / 2.0)
        marker.pose.orientation.w = math.cos(self.yaw / 2.0)

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color (반투명 빨강)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.25

        marker.points.clear()

        # FoV 부채꼴 생성 (센서 로컬 +X 기준)
        angles = [
            -self.fov / 2.0 + i * self.fov / self.segments
            for i in range(self.segments + 1)
        ]

        origin = Point(x=0.0, y=0.0, z=0.0)

        for i in range(len(angles) - 1):
            a1 = angles[i]
            a2 = angles[i + 1]

            p1 = Point(
                x=self.range_max * math.cos(a1),
                y=self.range_max * math.sin(a1),
                z=0.0
            )

            p2 = Point(
                x=self.range_max * math.cos(a2),
                y=self.range_max * math.sin(a2),
                z=0.0
            )

            marker.points.append(origin)
            marker.points.append(p1)
            marker.points.append(p2)

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = ToFFoVMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

