#!/usr/bin/env python3

import math
import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Float64

from sailboat_physics.vector2 import Vector2

from sailboat_physics.dynamics import (
    compute_true_wind,
    compute_apparent_wind,
    compute_angle_of_attack,
    compute_sail_force,
    apply_water_drag,
    apply_keel_damping,
    compute_forward_speed,
    compute_rudder_yaw_rate
)


class BoatPhysicsNode(Node):

    def __init__(self):

        super().__init__('boat_physics_node')

        # POSITION

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.yaw = 0.0

        # VELOCITY

        self.velocity = Vector2(
            0.0,
            0.0
        )

        self.yaw_rate = 0.0

        # WIND

        self.wind_speed = 0.0
        self.wind_direction = 0.0

        # CONTROLS

        self.sail_angle = 0.0
        self.rudder_angle = 0.0

        # SIMULATION

        self.dt = 0.05

        # INIT

        self.initialize_pose()

        # SUBSCRIBERS

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

        # TIMER

        self.timer = self.create_timer(
            self.dt,
            self.update_motion
        )

        self.get_logger().info(
            'Boat physics node started.'
        )

    def initialize_pose(self):

        try:

            result = subprocess.run(
                [
                    'gz',
                    'topic',
                    '-e',
                    '-n',
                    '1',
                    '-t',
                    '/world/lake_balaton/dynamic_pose/info'
                ],
                capture_output=True,
                text=True,
                timeout=3
            )

            output = result.stdout.splitlines()

            inside_sailboat = False
            inside_position = False

            for line in output:

                line = line.strip()

                if 'name: "sailboat"' in line:
                    inside_sailboat = True

                if (
                    inside_sailboat and
                    line.startswith('position')
                ):
                    inside_position = True

                if inside_position:

                    if line.startswith('x:'):
                        self.x = float(
                            line.split(':')[1]
                        )

                    elif line.startswith('y:'):
                        self.y = float(
                            line.split(':')[1]
                        )

                    elif line.startswith('z:'):
                        self.z = float(
                            line.split(':')[1]
                        )

                if (
                    inside_sailboat and
                    line.startswith('orientation')
                ):
                    break

        except Exception as e:

            self.get_logger().error(
                str(e)
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
    # MAIN UPDATE LOOP
    # =====================================================

    def update_motion(self):

        # TRUE WIND

        true_wind = compute_true_wind(
            self.wind_speed,
            self.wind_direction
        )

        # APPARENT WIND

        apparent_wind = compute_apparent_wind(
            true_wind,
            self.velocity
        )

        # ANGLE OF ATTACK

        angle_of_attack = compute_angle_of_attack(
            apparent_wind.direction(),
            self.yaw,
            self.sail_angle
        )

        # SAIL FORCE

        sail_force = compute_sail_force(
            apparent_wind,
            angle_of_attack
        )

        # ACCELERATION

        boat_mass = 6.44

        acceleration = (
            sail_force *
            (1.0 / boat_mass)
        )

        # VELOCITY INTEGRATION

        self.velocity += (
            acceleration *
            self.dt
        )

        # WATER DRAG

        self.velocity = apply_water_drag(
            self.velocity
        )

        # KEEL DAMPING

        self.velocity = apply_keel_damping(
            self.velocity,
            self.yaw,
            self.dt
        )

        # FORWARD SPEED

        forward_speed = compute_forward_speed(
            self.velocity,
            self.yaw
        )

        # RUDDER

        target_yaw_rate = compute_rudder_yaw_rate(
            self.rudder_angle,
            forward_speed
        )

        # YAW DYNAMICS

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

        # POSITION INTEGRATION

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

        # QUATERNION

        qw = math.cos(
            self.yaw / 2.0
        )

        qz = math.sin(
            self.yaw / 2.0
        )

        # GAZEBO REQUEST

        request = f'''
name: "sailboat"
position {{
  x: {self.x}
  y: {self.y}
  z: {self.z}
}}
orientation {{
  z: {qz}
  w: {qw}
}}
'''

        # SEND

        try:

            subprocess.run(
                [
                    'gz',
                    'service',

                    '-s',
                    '/world/lake_balaton/set_pose',

                    '--reqtype',
                    'gz.msgs.Pose',

                    '--reptype',
                    'gz.msgs.Boolean',

                    '--timeout',
                    '1000',

                    '--req',
                    request
                ],
                capture_output=True,
                text=True
            )

        except Exception as e:

            self.get_logger().error(
                str(e)
            )


def main(args=None):

    rclpy.init(args=args)

    node = BoatPhysicsNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()