#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_msgs.msg import Float32


# =========================================================
# UTILS
# =========================================================

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# =========================================================
# AUTONOMY NODE
# =========================================================

class SailboatAutonomy(Node):
    def __init__(self):
        super().__init__('sailboat_autonomy')

        # STATE
        self.x = None
        self.y = None
        self.heading = 0.0
        self.apparent_wind_dir = 0.0

        # WAYPOINTS
        self.waypoints = [
            (-505.0, 180.0),
            (-515.0, 180.0),
            (-515.0, 200.0),
            (-505.0, 200.0),
        ]
        self.current_wp = 0
        self.origin_lat = None
        self.origin_lon = None
        self.turn_sign = 1.0
        self.world_origin_x = -520.0
        self.world_origin_y = 191.0

        # RUDDER PID
        self.heading_integral = 0.0
        self.prev_heading_error = 0.0
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.4

        # SAIL PID
        self.sail_integral = 0.0
        self.prev_sail_error = 0.0
        self.sail_kp = 0.8
        self.sail_ki = 0.0
        self.sail_kd = 1.2

        # SUBSCRIBERS
        self.create_subscription(
            NavSatFix,
            '/boat/gps/data',
            self.gps_callback,
            10
        )
        self.create_subscription(
            Imu,
            '/boat/imu/data',
            self.imu_callback,
            10
        )
        self.create_subscription(
            Float32,
            '/sensor/apparent_wind/direction',
            self.wind_callback,
            10
        )

        # PUBLISHERS
        self.rudder_pub = self.create_publisher(
            Float64,
            '/rudder_pos',
            10
        )
        self.sail_pub = self.create_publisher(
            Float64,
            '/baum_pos',
            10
        )

        # TIMER
        self.timer = self.create_timer(0.1, self.update)

    # =====================================================
    # CALLBACKS
    # =====================================================

    def gps_callback(self, msg):
        # store local ENU origin
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
        # Earth radius
        R = 6378137.0
        # delta coordinates
        dlat = math.radians(msg.latitude - self.origin_lat)
        dlon = math.radians(msg.longitude - self.origin_lon)
        # local ENU approximation
        local_y = dlat * R
        local_x = (
            dlon * R *
            math.cos(
                math.radians(
                    self.origin_lat
                )
            )
        )
        # convert ENU local frame into Gazebo world frame
        self.x = self.world_origin_x + local_x
        self.y = self.world_origin_y + local_y

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)

    def wind_callback(self, msg):
        self.apparent_wind_dir = math.radians(msg.data)

    def calc_angle_to_normal(self, sail_angle, apparent_wind):
        # sail normal
        if sail_angle > 0.0:
            normal = sail_angle - math.pi / 2.0
        else:
            normal = sail_angle + math.pi / 2.0
        # angle difference
        angle = normalize_angle(apparent_wind - normal)
        return abs(angle)

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def update(self):
        if self.x is None:
            return

        # TARGET WAYPOINT
        tx, ty = self.waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        distance = math.hypot(dx, dy)

        # waypoint reached
        if distance < 3.0:
            self.current_wp += 1
            if self.current_wp >= len(self.waypoints):
                self.current_wp = 0
            return

        # TARGET HEADING
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.heading)

        # avoid oscillation around 180 deg
        if abs(math.degrees(heading_error)) > 170:
            heading_error = math.radians(179) * self.turn_sign
        else:
            self.turn_sign = (
                1.0 if heading_error > 0.0
                else -1.0
            )

        # PID RUDDER CONTROLLER
        dt = 0.1
        # integral
        self.heading_integral += heading_error * dt
        # derivative
        heading_derivative = (heading_error - self.prev_heading_error) / dt
        # PID
        rudder = -(
            self.kp * heading_error +
            self.ki * self.heading_integral +
            self.kd * heading_derivative
        )
        # store previous
        self.prev_heading_error = heading_error
        # clamp
        rudder = max(-0.4, min(0.4, rudder))

        # SAIL PID CONTROLLER
        # apparent wind FROM direction
        rel_wind = normalize_angle(self.apparent_wind_dir)
        target_angle = math.radians(85)
        current_angle = self.calc_angle_to_normal(
            sail_angle=0.0,
            apparent_wind=rel_wind
        )
        sail_error = target_angle - current_angle
        # PID
        self.sail_integral += sail_error * dt
        sail_derivative = (sail_error - self.prev_sail_error) / dt
        sail_output = (
            self.sail_kp * sail_error +
            self.sail_ki * self.sail_integral +
            self.sail_kd * sail_derivative
        )
        self.prev_sail_error = sail_error
        # convert to sail angle
        sail_angle = abs(sail_output)
        # clamp
        sail_angle = max(
            math.radians(0.1),
            min(
                math.radians(90),
                sail_angle
            )
        )

        # PUBLISH
        rudder_msg = Float64()
        rudder_msg.data = rudder
        sail_msg = Float64()
        sail_msg.data = sail_angle
        self.rudder_pub.publish(rudder_msg)
        self.sail_pub.publish(sail_msg)

        # DEBUG
        print("\n====================")
        print("TARGET WP :", self.current_wp)
        print("DISTANCE  :", round(distance, 2))
        print("HEADING ERR:", round(math.degrees(heading_error), 2))
        print("RUDDER:", round(rudder, 2))
        print("SAIL:", round(math.degrees(sail_angle), 2))


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = SailboatAutonomy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
