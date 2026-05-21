#!/usr/bin/env python3

import math
import subprocess
import rclpy
from rclpy.node import Node


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

        # MOTION PARAMETERS
        self.forward_velocity = 1.0
        self.yaw_rate = 0.0

        # TIMER
        self.dt = 0.05
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

    def update_motion(self):
        """
        Updates the mostion of the boat.
        """
        # SIMPLE KINEMATIC MODEL
        self.x += (
            self.forward_velocity *
            math.cos(self.yaw) *
            self.dt
        )
        self.y += (
            self.forward_velocity *
            math.sin(self.yaw) *
            self.dt
        )
        self.yaw += (
            self.yaw_rate *
            self.dt
        )

        # YAW -> QUATERNION
        qw = math.cos(self.yaw / 2.0)
        qz = math.sin(self.yaw / 2.0)

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
