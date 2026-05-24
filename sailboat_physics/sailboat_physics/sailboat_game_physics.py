#!/usr/bin/env python3

import math
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64


# =========================================================
# SIMPLE VECTOR2
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

class SailboatGamePhysics(Node):

    def __init__(self):

        super().__init__(
            'sailboat_game_physics'
        )

        # =================================================
        # POSITION
        # =================================================

        self.x = -520.0
        self.y = 191.0
        self.z = 0.0

        self.yaw = 0.0

        # =================================================
        # VELOCITY
        # =================================================

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

        # actual boom position
        self.sail_angle = 0.0
        self.sail_sign = 1.0

        # rope length / max boom angle
        self.sheet_length = math.radians(80)
        self.rudder_angle = 0.0

        # =================================================
        # PARAMETERS
        # =================================================

        self.dt = 0.05

        self.boat_mass = 6.44

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

        self.actual_sail_pub = self.create_publisher(
            Float64,
            '/actual_baum_pos',
            10
        )

        # =================================================
        # GAZEBO TRANSPORT
        # =================================================

        self.gz_node = GzNode()

        # =================================================
        # TIMER
        # =================================================

        self.timer = self.create_timer(
            self.dt,
            self.update_physics
        )

        self.get_logger().info(
            'Sailboat Game Physics Started'
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

        # rope length only
        self.sheet_length = abs(msg.data)

    def rudder_callback(self, msg):

        self.rudder_angle = msg.data

    def calc_efficiency(self, apparent_dir):

        # =====================================================
        # GLOBAL SAIL DIRECTION
        # =====================================================

        sail_dir = self.yaw + self.sail_angle

        # =====================================================
        # APPARENT WIND VECTOR
        # =====================================================

        aw_x = math.cos(apparent_dir)
        aw_y = math.sin(apparent_dir)

        # =====================================================
        # ACTIVE SAIL NORMAL
        # =====================================================

        # choose inward-facing normal

        if self.sail_angle > 0.0:

            normal_dir = sail_dir - math.pi / 2.0

        else:

            normal_dir = sail_dir + math.pi / 2.0

        normal_x = math.cos(normal_dir)
        normal_y = math.sin(normal_dir)

        # =====================================================
        # ANGLE BETWEEN NORMAL AND WIND
        # =====================================================

        dot = (
            aw_x * normal_x +
            aw_y * normal_y
        )

        dot = max(-1.0, min(1.0, dot))

        angle = math.acos(dot)

        angle_deg = math.degrees(angle)

        # =====================================================
        # EFFICIENCY
        # =====================================================

        # 0°   -> luffing
        # 90°  -> strongest

        efficiency = math.sin(angle)

        efficiency = max(
            0.0,
            min(1.0, efficiency)
        ) if angle_deg <= 90 else 0.0

        # =====================================================
        # DEBUG
        # =====================================================

        print(
            "ANGLE TO NORMAL :",
            round(angle_deg, 2)
        )

        print(
            "TRIM EFFICIENCY :",
            round(efficiency, 3)
        )

        return efficiency


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

        relative_deg = math.degrees(
            relative_wind
        )

        # =================================================
        # BOOM SIDE SELECTION
        # =================================================

        # dead downwind singularity:
        # keep previous side

        deadzone = math.radians(15)

        if abs(relative_wind) > deadzone:

            if relative_wind > 0.0:

                self.sail_sign = -1.0

            else:

                self.sail_sign = 1.0

        # =================================================
        # BOOM TARGET
        # =================================================

        target_boom = (

            self.sail_sign *
            self.sheet_length
        )

        # smooth motion

        sail_response = 4.0

        self.sail_angle += (

            target_boom -
            self.sail_angle

        ) * sail_response * self.dt

        # =================================================
        # WIND EFFICIENCY
        # =================================================

        # strongest on beam reach
        # weaker upwind/downwind

        wind_efficiency = abs(
            math.sin(relative_wind)
        )

        angle = abs(relative_deg)

        # 0 = dead downwind
        # 180 = headwind

        if angle < 20:

            wind_efficiency = 0.25

        elif angle < 45:

            wind_efficiency = 0.9

        elif angle < 80:

            wind_efficiency = 1.0

        elif angle < 120:

            wind_efficiency = 0.85

        elif angle < 160:

            wind_efficiency = 0.95

        elif angle < 172:

            wind_efficiency = 0.6

        else:

            wind_efficiency = 0.0

        # =================================================
        # TRIM EFFICIENCY
        # =================================================

        trim_efficiency = self.calc_efficiency(
            apparent_dir
        )

        # =================================================
        # TOTAL SAIL POWER
        # =================================================

        sail_power = (

            apparent_speed *
            apparent_speed *

            wind_efficiency *
            trim_efficiency
        )

        sail_gain = 0.45

        thrust = (
            sail_power *
            sail_gain
        )

        # =================================================
        # BOAT DIRECTIONS
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
        # SAIL FORCE
        # =================================================

        sail_force = (
            forward *
            thrust
        )

        # =================================================
        # ACCELERATION
        # =================================================

        acceleration = (
            sail_force *
            (1.0 / self.boat_mass)
        )

        self.velocity += (
            acceleration *
            self.dt
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
        # WATER DRAG
        # =================================================

        forward_drag = 0.4
        reverse_drag = 4.0
        sideways_drag = 3.5

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

        sideways_force = (

            -sideways_drag *
            sideways_speed *
            abs(sideways_speed)
        )

        drag_force = (

            forward * forward_force +
            sideways * sideways_force
        )

        self.velocity += (
            drag_force *
            self.dt
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

        self.velocity += (
            keel_force *
            self.dt
        )

        # =================================================
        # RUDDER
        # =================================================

        rudder_gain = 1.8

        target_yaw_rate = (

            -self.rudder_angle *

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
        # POSITION
        # =================================================

        self.x += (
            self.velocity.x *
            self.dt
        )

        self.y += (
            self.velocity.y *
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
        # QUATERNION
        # =================================================

        qw = math.cos(
            self.yaw / 2.0
        )

        qz = math.sin(
            self.yaw / 2.0
        )

        # =================================================
        # DEBUG
        # =================================================

        print("\n========================")

        print(
            "TRUE WIND DIR :",
            round(math.degrees(
                self.wind_direction
            ), 2)
        )

        print(
            "BOAT HEADING  :",
            round(math.degrees(
                self.yaw
            ), 2)
        )

        print(
            "APP WIND DIR  :",
            round(math.degrees(
                apparent_dir
            ), 2)
        )

        print(
            "REL WIND      :",
            round(relative_deg, 2)
        )

        print(
            "boat speed    :",
            round(
                self.velocity.magnitude(),
                3
            )
        )

        print(
            "wind eff      :",
            round(
                wind_efficiency,
                3
            )
        )

        print(
            "sail force    :",
            round(
                thrust,
                3
            )
        )

        print(
            "boom angle    :",
            round(
                math.degrees(
                    self.sail_angle
                ),
                2
            )
        )

        print(
            "sheet length  :",
            round(
                math.degrees(
                    self.sheet_length
                ),
                2
            )
        )

        print("========================")

        # =================================================
        # VISUAL BOOM UPDATE
        # =================================================

        actual_msg = Float64()

        actual_msg.data = self.sail_angle

        self.actual_sail_pub.publish(actual_msg)

        # =================================================
        # SEND POSE TO GAZEBO
        # =================================================

        req = Pose()

        req.name = 'sailboat'

        req.position.x = self.x
        req.position.y = self.y
        req.position.z = self.z

        req.orientation.z = qz
        req.orientation.w = qw

        success, response = self.gz_node.request(
            '/world/lake_balaton/set_pose',
            req,
            Pose,
            Boolean,
            100
        )
        if not success:

            print("SET_POSE REQUEST FAILED")

        elif not response.data:

            print("SET_POSE REJECTED")

    #     # =================================================
    #     # SEND POSE TO GAZEBO
    #     # =================================================

    #     request = f'''
    # name: "sailboat"
    # position {{
    # x: {self.x}
    # y: {self.y}
    # z: {self.z}
    # }}
    # orientation {{
    # z: {qz}
    # w: {qw}
    # }}
    # '''

    #     try:

    #         subprocess.run(
    #             [
    #                 'gz',
    #                 'service',

    #                 '-s',
    #                 '/world/lake_balaton/set_pose',

    #                 '--reqtype',
    #                 'gz.msgs.Pose',

    #                 '--reptype',
    #                 'gz.msgs.Boolean',

    #                 '--timeout',
    #                 '1000',

    #                 '--req',
    #                 request
    #             ],
    #             capture_output=True,
    #             text=True
    #         )

    #     except Exception as e:

    #         self.get_logger().error(
    #             str(e)
    #         )

# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = SailboatGamePhysics()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()