"""Estimate apparent wind in the boat body frame."""

import math

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class ApparentWindSensor(Node):
    """Publish apparent wind direction and speed from wind, velocity, and IMU."""

    def __init__(self) -> None:
        super().__init__('apparent_wind_sensor')

        self.true_wind_dir_sub = self.create_subscription(
            Float32,
            '/vrx/debug/wind/direction',
            self.true_wind_dir_callback,
            10
        )
        self.true_wind_speed_sub = self.create_subscription(
            Float32,
            '/vrx/debug/wind/speed',
            self.true_wind_speed_callback,
            10
        )
        self.velocity_sub = self.create_subscription(
            Vector3,
            '/boat/velocity',
            self.velocity_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/boat/imu/data',
            self.imu_callback,
            10
        )

        self.apparent_wind_dir_pub = self.create_publisher(
            Float32,
            '/sensor/apparent_wind/direction',
            10
        )
        self.apparent_wind_speed_pub = self.create_publisher(
            Float32,
            '/sensor/apparent_wind/speed',
            10
        )

        self.true_wind_dir: float | None = None
        self.true_wind_speed: float | None = None
        self.boat_heading: float | None = None
        self.boat_vel_x = 0.0
        self.boat_vel_y = 0.0

        self.timer = self.create_timer(0.1, self.compute_apparent_wind)

    def true_wind_dir_callback(self, msg: Float32) -> None:
        self.true_wind_dir = math.radians(msg.data)

    def true_wind_speed_callback(self, msg: Float32) -> None:
        self.true_wind_speed = msg.data

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.boat_heading = math.atan2(siny_cosp, cosy_cosp)

    def velocity_callback(self, msg: Vector3) -> None:
        self.boat_vel_x = msg.x
        self.boat_vel_y = msg.y

    def compute_apparent_wind(self) -> None:
        if None in (
            self.true_wind_dir,
            self.true_wind_speed,
            self.boat_heading
        ):
            return

        # 1. Global True Wind Vector (ENU)
        v_wind_x = self.true_wind_speed * math.cos(self.true_wind_dir)
        v_wind_y = self.true_wind_speed * math.sin(self.true_wind_dir)

        # 2. Global Apparent Wind Vector
        v_app_global_x = v_wind_x - self.boat_vel_x
        v_app_global_y = v_wind_y - self.boat_vel_y

        # 3. Transform Global Apparent Wind into Boat Body Frame (X-Forward, Y-Left)
        v_app_body_x = (
            v_app_global_x * math.cos(self.boat_heading) +
            v_app_global_y * math.sin(self.boat_heading)
        )
        v_app_body_y = (
            -v_app_global_x * math.sin(self.boat_heading) +
            v_app_global_y * math.cos(self.boat_heading)
        )

        # 4. Calculate Apparent Wind Speed
        apparent_speed = math.hypot(v_app_body_x, v_app_body_y)

        # 5. Calculate Marine Apparent Wind Direction (Angle coming FROM)
        apparent_dir_from = math.atan2(-v_app_body_y, -v_app_body_x)

        dir_msg = Float32()
        dir_msg.data = math.degrees(apparent_dir_from)
        self.apparent_wind_dir_pub.publish(dir_msg)

        speed_msg = Float32()
        speed_msg.data = apparent_speed
        self.apparent_wind_speed_pub.publish(speed_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApparentWindSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
