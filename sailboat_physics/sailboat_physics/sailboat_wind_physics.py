#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64

from gz.msgs10.entity_wrench_pb2 import EntityWrench


# =========================================================
# VECTOR2
# =========================================================

class Vector2:

    def __init__(self, x=0.0, y=0.0):

        self.x = x
        self.y = y

    def magnitude(self):

        return math.hypot(
            self.x,
            self.y
        )

    def direction(self):

        return math.atan2(
            self.y,
            self.x
        )

    def dot(self, other):

        return (
            self.x * other.x +
            self.y * other.y
        )

    def __add__(self, other):

        return Vector2(
            self.x + other.x,
            self.y + other.y
        )

    def __sub__(self, other):

        return Vector2(
            self.x - other.x,
            self.y - other.y
        )

    def __mul__(self, scalar):

        return Vector2(
            self.x * scalar,
            self.y * scalar
        )


# =========================================================
# UTILS
# =========================================================

def normalize_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


# =========================================================
# MAIN NODE
# =========================================================

class SailboatForcePhysics(Node):

    def __init__(self):

        super().__init__(
            'sailboat_force_physics'
        )

        # =================================================
        # STATE
        # =================================================

        self.yaw = 0.0

        self.velocity = Vector2(
            0.0,
            0.0
        )

        self.yaw_rate = 0.0

        # =================================================
        # WIND
        # =================================================

        self.wind_speed = 4.0
        self.wind_direction = math.radians(240)

        # =================================================
        # CONTROLS
        # =================================================

        self.sail_angle = 0.0
        self.rudder_angle = 0.0

        # =================================================
        # PARAMETERS
        # =================================================

        self.dt = 0.05

        self.boat_mass = 6.44

        # =================================================
        # GAZEBO WRENCH PUBLISHER
        # =================================================

        from gz.transport13 import Node as GzNode

        self.gz_node = GzNode()

        self.gz_pub = self.gz_node.advertise(
            '/world/lake_balaton/wrench',
            EntityWrench
        )

        # =================================================
        # SUBSCRIBERS
        # =================================================

        self.create_subscription(
            Float32,
            '/vrx/debug/wind/speed',
            self.wind_speed_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/vrx/debug/wind/direction',
            self.wind_direction_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/baum_pos',
            self.sail_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/rudder_pos',
            self.rudder_callback,
            10
        )

        # =================================================
        # TIMER
        # =================================================

        self.timer = self.create_timer(
            self.dt,
            self.update_physics
        )

        self.get_logger().info(
            'Sailboat Force Physics Started'
        )

    # =====================================================
    # CALLBACKS
    # =====================================================

    def wind_speed_callback(self, msg):

        self.wind_speed = msg.data

    def wind_direction_callback(self, msg):

        self.wind_direction = math.radians(
            msg.data
        )

    def sail_callback(self, msg):

        self.sail_angle = msg.data

    def rudder_callback(self, msg):

        self.rudder_angle = msg.data

    # =====================================================
    # MAIN PHYSICS
    # =====================================================

    def update_physics(self):

        # =================================================
        # TRUE WIND
        # =================================================

        true_wind = Vector2(

            self.wind_speed *
            math.cos(self.wind_direction),

            self.wind_speed *
            math.sin(self.wind_direction)
        )

        # =================================================
        # BOAT AXES
        # =================================================

        forward = Vector2(

            math.cos(self.yaw),
            math.sin(self.yaw)
        )

        sideways = Vector2(

            -math.sin(self.yaw),
             math.cos(self.yaw)
        )

        # =================================================
        # APPARENT WIND
        # =================================================

        apparent_wind = (
            true_wind -
            self.velocity
        )

        apparent_speed = (
            apparent_wind.magnitude()
        )

        apparent_dir = (
            apparent_wind.direction()
        )

        # =================================================
        # RELATIVE WIND
        # =================================================

        relative_wind = normalize_angle(

            apparent_dir -
            self.yaw
        )

        # =================================================
        # SAIL EFFICIENCY
        # =================================================

        wind_efficiency = max(
            0.15,
            abs(math.sin(relative_wind))
        )

        # =================================================
        # SAIL TRIM
        # =================================================

        sail_error = normalize_angle(

            relative_wind -
            self.sail_angle
        )

        trim_efficiency = abs(
            math.cos(sail_error)
        )

        # =================================================
        # LOCAL VELOCITIES
        # =================================================

        forward_speed = (
            self.velocity.dot(forward)
        )

        sideways_speed = (
            self.velocity.dot(sideways)
        )

        # =================================================
        # WIND PUSH
        # =================================================

        wind_push = -apparent_wind.dot(
            forward
        )

        wind_push = max(
            0.0,
            wind_push
        )

        # =================================================
        # SAIL POWER
        # =================================================

        sail_power = (

            wind_push *

            wind_efficiency *

            trim_efficiency
        )

        sail_gain = 8.0

        thrust = (
            sail_power *
            sail_gain
        )

        # =================================================
        # DRAG
        # =================================================

        forward_drag = 0.4
        reverse_drag = 2.5
        sideways_drag = 3.0

        # FORWARD / REVERSE

        if forward_speed >= 0.0:

            forward_force = (

                -forward_drag *

                forward_speed *

                abs(forward_speed)
            )

        else:

            forward_force = (

                reverse_drag *

                forward_speed *

                abs(forward_speed)
            )

        # SIDEWAYS

        sideways_force = (

            -sideways_drag *

            sideways_speed *

            abs(sideways_speed)
        )

        # =================================================
        # TOTAL FORCE
        # =================================================

        force_x = (

            forward.x * thrust +

            forward.x * forward_force +

            sideways.x * sideways_force
        )

        force_y = (

            forward.y * thrust +

            forward.y * forward_force +

            sideways.y * sideways_force
        )

        # =================================================
        # KEEL
        # =================================================

        keel_strength = 2.5

        keel_force = (

            sideways *

            -sideways_speed *

            keel_strength
        )

        force_x += keel_force.x
        force_y += keel_force.y

        # =================================================
        # RUDDER
        # =================================================

        rudder_gain = 2.0

        target_yaw_rate = (

            self.rudder_angle *

            rudder_gain *

            forward_speed
        )

        yaw_gain = 2.0

        yaw_error = (
            target_yaw_rate -
            self.yaw_rate
        )

        yaw_acceleration = (
            yaw_error *
            yaw_gain
        )

        self.yaw_rate += (
            yaw_acceleration *
            self.dt
        )

        # =================================================
        # INTERNAL STATE UPDATE
        # =================================================

        acceleration = Vector2(
            force_x / self.boat_mass,
            force_y / self.boat_mass
        )

        self.velocity += (
            acceleration *
            self.dt
        )

        self.yaw += (
            self.yaw_rate *
            self.dt
        )

        self.yaw = normalize_angle(
            self.yaw
        )

        # =================================================
        # BUILD WRENCH MESSAGE
        # =================================================

        msg = EntityWrench()

        msg.entity.name = "sailboat::base_link"
        msg.entity.type = 3

        msg.wrench.force.x = float(force_x)
        msg.wrench.force.y = float(force_y)
        msg.wrench.force.z = 0.0

        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = float(
            self.yaw_rate * 4.0
        )

        # =================================================
        # PUBLISH
        # =================================================

        self.gz_pub.publish(msg)

        # =================================================
        # DEBUG
        # =================================================

        print("\n========================")

        print(
            "heading:",
            round(math.degrees(self.yaw), 2)
        )

        print(
            "boat speed:",
            round(self.velocity.magnitude(), 3)
        )

        print(
            "forward speed:",
            round(forward_speed, 3)
        )

        print(
            "sideways speed:",
            round(sideways_speed, 3)
        )

        print(
            "relative wind:",
            round(math.degrees(relative_wind), 2)
        )

        print(
            "thrust:",
            round(thrust, 3)
        )

        print(
            "force:",
            round(force_x, 3),
            round(force_y, 3)
        )

        print("========================")


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = SailboatForcePhysics()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()