#!/usr/bin/env python3

import math
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64


class BoatMotionNode(Node):
    def __init__(self):
        super().__init__('boat_motion_node')

        # BOAT STATE
        # initialize
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # INITIALIZE FROM GAZEBO
        self.initialize_pose()

        # WIND STATE
        self.wind_speed = 0.0
        self.wind_direction = 0.0

        # CONTROL SURFACES
        self.sail_angle = 0.0
        self.rudder_angle = 0.0

        # MOTION PARAMETERS
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.target_yaw_rate = 0.0

        # SUBSRCIBERS
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
        self.dt = 0.2
        self.timer = self.create_timer(self.dt, self.update_motion)
        self.get_logger().info('Boat motion node started.')


    def initialize_pose(self):
        """
        Initializes the pose of the sailboat from Gazebo.
        The pose info from the subprocess looks like this:
        header {
            stamp {
            }
        }
        pose {
            name: "sailboat"
            id: 7
            position {
                x: -520
                y: 191
            }
            orientation {
                w: 1
            }
        } ... other objects
        """
        self.get_logger().info('Reading initial boat pose from Gazebo...')
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
                # ENTER SAILBOAT BLOCK
                if 'name: "sailboat"' in line:
                    inside_sailboat = True
                # ENTER POSITION BLOCK
                if inside_sailboat and line.startswith('position'):
                    inside_position = True
                # READ POSITION
                if inside_position:
                    if line.startswith('x:'):
                        self.x = float(line.split(':')[1])
                    elif line.startswith('y:'):
                        self.y = float(line.split(':')[1])
                    elif line.startswith('z:'):
                        self.z = float(line.split(':')[1])

                # END OF BOAT POSE
                if inside_sailboat and line.startswith('orientation'):
                    self.yaw = 0.0
                    break
            self.get_logger().info(
                f'Initial pose loaded: '
                f'x={self.x:.2f}, '
                f'y={self.y:.2f}, '
                f'z={self.z:.2f}'
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while reading Gazebo pose.')
        except Exception as e:
            self.get_logger().error(str(e))

    # WIND CALLBACKS
    def wind_speed_callback(self, msg):
        self.wind_speed = msg.data

    def wind_direction_callback(self, msg):
        self.wind_direction = math.radians(msg.data)

    # COPNTROL SURFACE CALLBACKS
    def sail_callback(self, msg):
        self.sail_angle = msg.data

    def rudder_callback(self, msg):
        self.rudder_angle = msg.data

    def update_motion(self):
        """
        Updates the motion of the sailboat.
        """
        # TRUE WIND VECTOR
        wind_x = self.wind_speed * math.cos(self.wind_direction)
        wind_y = self.wind_speed * math.sin(self.wind_direction)

        # BOAT VELOCITY VECTOR
        boat_vx = self.vx
        boat_vy = self.vy

        # APPARENT WIND VECTOR
        app_wind_x = wind_x - boat_vx
        app_wind_y = wind_y - boat_vy

        # APPARENT WIND SPEED + DIRECTION
        app_wind_speed = math.hypot(app_wind_x, app_wind_y)
        app_wind_direction = math.atan2(app_wind_y, app_wind_x)

        # ANGLE OF ATTACK
        angle_of_attack = app_wind_direction - self.yaw - self.sail_angle

        # normalize to [-pi, pi]
        angle_of_attack = math.atan2(
            math.sin(angle_of_attack),
            math.cos(angle_of_attack)
        )

        # LIFT / DRAG COEFFICIENTS
        lift_coefficient = math.sin(2.0 * angle_of_attack)
        drag_coefficient = 1.0 - math.cos(angle_of_attack)

        # FORCE MAGNITUDES
        lift_gain = 0.8
        drag_gain = 0.3
        lift_force = (app_wind_speed ** 2) * lift_coefficient * lift_gain

        drag_force = (app_wind_speed ** 2) * drag_coefficient * drag_gain

        # DRAG DIRECTION
        drag_dir_x = math.cos(app_wind_direction)
        drag_dir_y = math.sin(app_wind_direction)

        # LIFT DIRECTION
        lift_sign = 1.0
        if angle_of_attack < 0.0:
            lift_sign = -1.0
        lift_direction = app_wind_direction + lift_sign * math.pi / 2.0
        lift_dir_x = math.cos(lift_direction)
        lift_dir_y = math.sin(lift_direction)

        # TOTAL SAIL FORCE VECTOR
        force_x = lift_force * lift_dir_x + drag_force * drag_dir_x
        force_y = lift_force * lift_dir_y + drag_force * drag_dir_y

        # FORCE -> ACCELERATION
        boat_mass = 6.44
        ax = force_x / boat_mass
        ay = force_y / boat_mass

        # INTEGRATE VELOCITY
        self.vx += ax * self.dt
        self.vy += ay * self.dt

        # WATER DRAG
        water_drag = 0.98
        self.vx *= water_drag
        self.vy *= water_drag

        # FORWARD SPEED IN BODY FRAME
        forward_speed = (
            self.vx * math.cos(self.yaw) +
            self.vy * math.sin(self.yaw)
        )

        # KEEL SIDEWAYS DAMPING
        side_x = -math.sin(self.yaw)
        side_y = math.cos(self.yaw)
        side_velocity = self.vx * side_x + self.vy * side_y
        keel_strength = 0.9
        self.vx -= side_velocity * side_x * keel_strength * self.dt
        self.vy -= side_velocity * side_y * keel_strength * self.dt

        # RUDDER TURNING
        rudder_gain = 0.5
        self.target_yaw_rate = self.rudder_angle * rudder_gain * forward_speed

        # SMOOTH YAW DYNAMICS
        yaw_gain = 2.0
        yaw_error = self.target_yaw_rate - self.yaw_rate
        yaw_acceleration = yaw_error * yaw_gain
        self.yaw_rate += yaw_acceleration * self.dt

        # INTEGRATE POSITION
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.yaw += self.yaw_rate * self.dt

        # YAW -> QUATERNION
        qw = math.cos(self.yaw / 2.0)
        qz = math.sin(self.yaw / 2.0)

        # OPTIONAL DEBUG
        # self.get_logger().info(
        #     f'aws={app_wind_speed:.2f}, '
        #     f'aoa={math.degrees(angle_of_attack):.1f}, '
        #     f'vx={self.vx:.2f}, '
        #     f'vy={self.vy:.2f}'
        # )

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

        # SEND TO GAZEBO
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
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = BoatMotionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
