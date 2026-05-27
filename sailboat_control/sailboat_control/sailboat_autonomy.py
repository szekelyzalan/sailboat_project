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

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def calculate_sail_angle(self, rel_wind):
        # BOAT FORWARD VECTOR
        fx = 1.0
        fy = 0.0
        # WIND VECTOR (BLUE)
        wx = math.cos(rel_wind)
        wy = math.sin(rel_wind)
        # APPARENT VECTOR
        # forward - wind
        ax = fx - wx
        ay = fy - wy
        # FULL TAILWIND SINGULARITY
        magnitude = math.hypot(ax, ay)
        if magnitude < 1e-6:
            return math.radians(90)
        apparent_angle = math.atan2(ay, ax)
        sail_angle = (abs(apparent_angle) + math.radians(5))
        # CONVERT TO BAUM ANGLE
        sail_angle = math.pi / 2 - sail_angle
        sail_angle = max(math.radians(0), min(math.radians(90), sail_angle))
        return sail_angle


    def update(self):
        if self.x is None:
            return
        dt = 0.1
        # TARGET WAYPOINT
        tx, ty = self.waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        distance = math.hypot(dx, dy)
        # WAYPOINT REACHED
        if distance < 3.0:
            self.current_wp += 1
            if self.current_wp >= len(self.waypoints):
                self.current_wp = 0
            return
        # TARGET HEADING
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.heading)
        # 180 DEG SINGULARITY
        if abs(math.degrees(heading_error)) > 170:
            heading_error = math.radians(179) * self.turn_sign
        else:
            self.turn_sign = (
                1.0 if heading_error > 0.0
                else -1.0
            )

        # RUDDER PID
        self.heading_integral += heading_error * dt
        heading_derivative = (heading_error - self.prev_heading_error) / dt
        rudder = -(
            self.kp * heading_error +
            self.ki * self.heading_integral +
            self.kd * heading_derivative
        )
        self.prev_heading_error = heading_error
        rudder = max(-0.4, min(0.4, rudder))
        # SAIL CONTROL
        rel_wind = normalize_angle(self.apparent_wind_dir)
        sail_angle = self.calculate_sail_angle(rel_wind)
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
        print("SAIL ANGLE:", math.degrees(sail_angle))


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
