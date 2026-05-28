#!/usr/bin/env python3

"""Simplified wind-driven dynamics for the Gazebo sailboat model."""

import math
import sys

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from std_msgs.msg import Float64


class Vector2:
    """Minimal 2D vector helper for the planar dynamics model."""

    def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
        self.x = x
        self.y = y

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)

    def direction(self) -> float:
        return math.atan2(self.y, self.x)

    def dot(self, other: 'Vector2') -> float:
        return (
            self.x * other.x +
            self.y * other.y
        )

    def __add__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(
            self.x + other.x,
            self.y + other.y
        )

    def __sub__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(
            self.x - other.x,
            self.y - other.y
        )

    def __mul__(self, scalar: float) -> 'Vector2':
        return Vector2(
            self.x * scalar,
            self.y * scalar
        )


def normalize_angle(angle: float) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def strip_debug_flags(args: list[str]) -> tuple[bool, list[str]]:
    debug = False
    clean_args = []
    for arg in args:
        if arg in ('--debug', '--verbose'):
            debug = True
        else:
            clean_args.append(arg)
    return debug, clean_args


def time_to_sec(time_msg: Time) -> float:
    return time_msg.sec + time_msg.nanosec * 1e-9


class SailboatGamePhysics(Node):
    """Advance the boat state when Gazebo simulation time is running."""

    def __init__(self, debug_default: bool = False) -> None:
        super().__init__('sailboat_game_physics')

        self.declare_parameter('debug', debug_default)
        self.declare_parameter('verbose', debug_default)
        self.debug_enabled = self.get_parameter('debug').value
        self.verbose_enabled = self.get_parameter('verbose').value

        self.x = -520.0
        self.y = 191.0
        self.z = 0.0
        self.yaw = 0.0

        self.velocity = Vector2(0.0, 0.0)
        self.yaw_rate = 0.0

        self.wind_speed = 4.0
        self.wind_direction = math.radians(40)

        self.sail_angle = 0.0
        self.sail_sign = 1.0

        self.sheet_length = math.radians(80)
        self.rudder_angle = 0.0

        self.dt = 0.05
        self.boat_mass = 6.44
        self.latest_clock: Time | None = None
        self.last_physics_clock: Time | None = None

        self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
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
        self.velocity_pub = self.create_publisher(
            Vector3,
            '/boat/velocity',
            10
        )

        self.gz_node = GzNode()

        self.timer = self.create_timer(
            self.dt,
            self.update_physics
        )
        if self.verbose_enabled:
            self.get_logger().info('Sailboat Game Physics Started')

    def clock_callback(self, msg: Clock) -> None:
        self.latest_clock = msg.clock

    def wind_speed_callback(self, msg: Float32) -> None:
        self.wind_speed = msg.data

    def wind_direction_callback(self, msg: Float32) -> None:
        self.wind_direction = math.radians(msg.data)

    def sail_callback(self, msg: Float64) -> None:
        self.sheet_length = abs(msg.data)

    def rudder_callback(self, msg: Float64) -> None:
        self.rudder_angle = msg.data

    def simulation_is_running(self) -> bool:
        if self.latest_clock is None:
            return False

        if self.last_physics_clock is None:
            self.last_physics_clock = Time()
            self.last_physics_clock.sec = self.latest_clock.sec
            self.last_physics_clock.nanosec = self.latest_clock.nanosec
            return False

        latest = time_to_sec(self.latest_clock)
        previous = time_to_sec(self.last_physics_clock)
        if latest <= previous:
            return False

        self.last_physics_clock.sec = self.latest_clock.sec
        self.last_physics_clock.nanosec = self.latest_clock.nanosec
        return True

    def debug_print(self, *args) -> None:
        if self.debug_enabled:
            print(*args)

    def calc_efficiency(self, relative_wind: float) -> float:
        # FORWARD VECTOR
        fx = 1.0
        fy = 0.0
        # WIND VECTOR
        wx = math.cos(relative_wind)
        wy = math.sin(relative_wind)
        # forward - wind
        ox = fx - wx
        oy = fy - wy
        magnitude = math.hypot(ox, oy)
        # FULL DOWNWIND SINGULARITY
        if magnitude < 1e-6:
            return abs(math.sin(self.sail_angle))
        # normalize forward - wind
        ox /= magnitude
        oy /= magnitude
        # SAIL NORMAL
        if relative_wind > 0.0:
            # wind from left
            # sail on right
            # inward normal points left
            normal_dir = self.sail_angle + math.pi / 2.0
        else:
            # wind from right
            # sail on left
            # inward normal points right
            normal_dir = self.sail_angle - math.pi / 2.0
        nx = math.cos(normal_dir)
        ny = math.sin(normal_dir)
        # ANGLE BETWEEN NORMAL AND FORWARD - WIND VECTOR
        dot = ox * nx + oy * ny
        dot = max(-1.0, min(1.0, dot))
        angle = math.pi - math.acos(dot)

        # EFFICIENCY
        efficiency = math.sin(angle)
        efficiency = max(
            0.0,
            min(1.0, efficiency)
        ) if angle <= math.pi/2 else 0.0

        self.debug_print('ANGLE TO NORMAL :', round(math.degrees(angle), 2))
        self.debug_print('TRIM EFFICIENCY :', round(efficiency, 3))

        return efficiency

    def update_physics(self) -> None:
        if not self.simulation_is_running():
            return

        # TRUE WIND
        true_wind = Vector2(
            self.wind_speed *
            math.cos(self.wind_direction),
            self.wind_speed *
            math.sin(self.wind_direction)
        )

        # APPARENT WIND
        apparent_wind = true_wind - self.velocity
        apparent_speed = apparent_wind.magnitude()
        apparent_dir = apparent_wind.direction()

        # RELATIVE WIND
        relative_wind = normalize_angle(apparent_dir - self.yaw)
        relative_deg = math.degrees(relative_wind)

        # dead downwind singularity: keep previous side
        deadzone = math.radians(15)
        if abs(relative_wind) > deadzone:
            if relative_wind > 0.0:
                self.sail_sign = -1.0
            else:
                self.sail_sign = 1.0

        target_baum = self.sail_sign * self.sheet_length
        # smooth motion
        sail_response = 4.0
        self.sail_angle += (target_baum - self.sail_angle) * sail_response * self.dt

        # WIND EFFICIENCY
        # strongest on beam reach
        # weaker upwind/downwind
        wind_efficiency = abs(math.sin(relative_wind))
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

        # TRIM EFFICIENCY
        trim_efficiency = self.calc_efficiency(relative_wind)

        # TOTAL SAIL POWER
        sail_power = (
            apparent_speed *
            apparent_speed *
            wind_efficiency *
            trim_efficiency
        )
        sail_gain = 0.45
        thrust = sail_power * sail_gain

        # BOAT DIRECTIONS
        forward = Vector2(
            math.cos(self.yaw),
            math.sin(self.yaw)
        )
        sideways = Vector2(
            -math.sin(self.yaw),
            math.cos(self.yaw)
        )

        # SAIL FORCE
        sail_force = forward * thrust

        # ACCELERATION
        acceleration = sail_force * (1.0 / self.boat_mass)
        self.velocity += acceleration * self.dt

        # LOCAL VELOCITIES
        forward_speed = self.velocity.dot(forward)
        sideways_speed = self.velocity.dot(sideways)

        # WATER DRAG
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
        self.velocity += drag_force * self.dt

        # KEEL
        keel_strength = 2.5
        keel_force = (
            sideways *
            -sideways_speed *
            keel_strength
        )
        self.velocity += keel_force * self.dt

        # RUDDER
        rudder_gain = 1.8
        target_yaw_rate = (
            -self.rudder_angle *
            rudder_gain *
            forward_speed
        )
        yaw_gain = 2.0
        yaw_error = target_yaw_rate - self.yaw_rate
        yaw_acceleration = yaw_error * yaw_gain
        self.yaw_rate += yaw_acceleration * self.dt

        # POSITION
        self.x += self.velocity.x * self.dt
        self.y += self.velocity.y * self.dt
        self.yaw += self.yaw_rate * self.dt
        self.yaw = normalize_angle(self.yaw)

        # QUATERNION
        qw = math.cos(self.yaw / 2.0)
        qz = math.sin(self.yaw / 2.0)

        if self.debug_enabled:
            print('\n========================')
            print(
                'TRUE WIND DIR :',
                round(math.degrees(
                    self.wind_direction
                ), 2)
            )
            print(
                'BOAT HEADING  :',
                round(math.degrees(
                    self.yaw
                ), 2)
            )
            print(
                'APP WIND DIR  :',
                round(math.degrees(
                    apparent_dir
                ), 2)
            )
            print(
                'REL WIND      :',
                round(relative_deg, 2)
            )
            print(
                'boat speed    :',
                round(
                    self.velocity.magnitude(),
                    3
                )
            )
            print(
                'wind eff      :',
                round(
                    wind_efficiency,
                    3
                )
            )
            print(
                'sail force    :',
                round(
                    thrust,
                    3
                )
            )
            print(
                'baum angle    :',
                round(
                    math.degrees(
                        self.sail_angle
                    ),
                    2
                )
            )
            print(
                'sheet length  :',
                round(
                    math.degrees(
                        self.sheet_length
                    ),
                    2
                )
            )
            print('========================')

        # VISUAL baum UPDATE
        actual_msg = Float64()
        actual_msg.data = self.sail_angle
        self.actual_sail_pub.publish(actual_msg)

        # PUBLISH VELOCITY
        vel_msg = Vector3()
        # self.velocity.x = 0.0
        # self.velocity.y = 0.0
        vel_msg.x = self.velocity.x
        vel_msg.y = self.velocity.y
        vel_msg.z = 0.0

        self.velocity_pub.publish(vel_msg)

        # SEND POSE TO GAZEBO
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
            self.get_logger().warning(
                'SET_POSE request failed',
                throttle_duration_sec=5.0
            )
        elif not response.data:
            self.get_logger().warning(
                'SET_POSE request rejected',
                throttle_duration_sec=5.0
            )


def main(args=None) -> None:
    raw_args = sys.argv[1:] if args is None else args
    debug_enabled, clean_args = strip_debug_flags(raw_args)
    rclpy.init(args=clean_args)
    node = SailboatGamePhysics(debug_default=debug_enabled)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
