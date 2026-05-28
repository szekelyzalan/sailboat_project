#!/usr/bin/env python3

"""Serve a local web dashboard populated from sailboat ROS topics."""

from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


def yaw_from_imu(msg: Imu) -> float:
    q = msg.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def gps_to_world(
    latitude: float,
    longitude: float,
    origin_lat: float,
    origin_lon: float,
    origin_x: float,
    origin_y: float
) -> tuple[float, float]:
    radius = 6378137.0
    dlat = math.radians(latitude - origin_lat)
    dlon = math.radians(longitude - origin_lon)
    local_y = dlat * radius
    local_x = dlon * radius * math.cos(math.radians(origin_lat))
    return origin_x + local_x, origin_y + local_y


class DashboardServer(ThreadingHTTPServer):
    """HTTP server with a reference to the ROS dashboard node."""

    def __init__(self, address, handler, node) -> None:
        super().__init__(address, handler)
        self.node = node


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve static dashboard assets and the live JSON state endpoint."""

    def do_GET(self) -> None:
        if self.path == '/' or self.path == '/index.html':
            self.serve_file('index.html', 'text/html; charset=utf-8')
            return

        if self.path == '/state':
            payload = json.dumps(self.server.node.snapshot()).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_error(404)

    def serve_file(self, filename: str, content_type: str) -> None:
        path = os.path.join(self.server.node.dashboard_dir, filename)
        with open(path, 'rb') as file:
            payload = file.read()

        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format, *_args) -> None:
        return


class SailboatDashboard(Node):
    """Collect simulation state and expose it over a local HTTP dashboard."""

    def __init__(self) -> None:
        super().__init__('sailboat_dashboard')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('gps_origin_lat', 46.9216)
        self.declare_parameter('gps_origin_lon', 17.8946)
        self.declare_parameter('gps_world_origin_x', 0.0)
        self.declare_parameter('gps_world_origin_y', 0.0)
        self.declare_parameter('max_track_points', 900)
        self.declare_parameter('max_lidar_points', 240)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.origin_lat = self.get_parameter('gps_origin_lat').value
        self.origin_lon = self.get_parameter('gps_origin_lon').value
        self.world_origin_x = self.get_parameter('gps_world_origin_x').value
        self.world_origin_y = self.get_parameter('gps_world_origin_y').value
        self.max_track_points = self.get_parameter('max_track_points').value
        self.max_lidar_points = self.get_parameter('max_lidar_points').value
        self.dashboard_dir = os.path.join(
            get_package_share_directory('sailboat_control'),
            'dashboard'
        )

        self.lock = threading.Lock()
        self.state = {
            'time': time.time(),
            'gps': None,
            'boat': None,
            'track': [],
            'heading_deg': None,
            'wind_dir_deg': None,
            'wind_speed': None,
            'apparent_wind_deg': None,
            'rudder_deg': None,
            'sail_deg': None,
            'actual_sail_deg': None,
            'course': None,
            'course_marks': [],
            'lap': 0,
            'autonomy': None,
            'buoy_relative': None,
            'lidar_points': [],
        }

        self.create_subscription(
            NavSatFix, '/boat/gps/data', self.gps_callback, 10
        )
        self.create_subscription(Imu, '/boat/imu/data', self.imu_callback, 10)
        self.create_subscription(
            Float32,
            '/vrx/debug/wind/direction',
            self.wind_dir_callback,
            10
        )
        self.create_subscription(
            Float32, '/vrx/debug/wind/speed', self.wind_speed_callback, 10
        )
        self.create_subscription(
            Float32,
            '/sensor/apparent_wind/direction',
            self.apparent_wind_callback,
            10
        )
        self.create_subscription(
            Float64, '/rudder_pos', self.rudder_callback, 10
        )
        self.create_subscription(
            Float64, '/baum_pos', self.sail_callback, 10
        )
        self.create_subscription(
            Float64, '/actual_baum_pos', self.actual_sail_callback, 10
        )
        self.create_subscription(
            Float64MultiArray,
            '/course/active_leg',
            self.course_callback,
            10
        )
        self.create_subscription(
            Float64MultiArray, '/course/marks', self.course_marks_callback, 10
        )
        self.create_subscription(
            String, '/autonomy/status', self.autonomy_callback, 10
        )
        self.create_subscription(
            PointStamped,
            '/perception/buoy/relative_position',
            self.buoy_callback,
            10
        )
        self.create_subscription(
            LaserScan, '/boat/lidar/scan', self.lidar_callback, 10
        )

        self.server = DashboardServer((self.host, self.port), DashboardHandler, self)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True
        )
        self.server_thread.start()
        self.get_logger().info(
            'Sailboat dashboard running at http://%s:%d' %
            (self.host, self.port)
        )

    def gps_callback(self, msg):
        x, y = gps_to_world(
            msg.latitude,
            msg.longitude,
            self.origin_lat,
            self.origin_lon,
            self.world_origin_x,
            self.world_origin_y
        )
        with self.lock:
            self.state['time'] = time.time()
            self.state['gps'] = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
            }
            self.state['boat'] = {'x': x, 'y': y}
            track = self.state['track']
            if (
                not track or
                math.hypot(track[-1]['x'] - x, track[-1]['y'] - y) > 0.15
            ):
                track.append({'x': x, 'y': y})
                del track[:-self.max_track_points]

    def imu_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['heading_deg'] = math.degrees(yaw_from_imu(msg))

    def wind_dir_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['wind_dir_deg'] = msg.data

    def wind_speed_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['wind_speed'] = msg.data

    def apparent_wind_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['apparent_wind_deg'] = msg.data

    def rudder_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['rudder_deg'] = math.degrees(msg.data)

    def sail_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['sail_deg'] = math.degrees(msg.data)

    def actual_sail_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['actual_sail_deg'] = math.degrees(msg.data)

    def course_callback(self, msg):
        if len(msg.data) < 8:
            return
        with self.lock:
            self.state['time'] = time.time()
            self.state['course'] = {
                'current_index': int(msg.data[0]),
                'current': {'x': msg.data[1], 'y': msg.data[2]},
                'rounding_side': 'port' if msg.data[3] > 0.0 else 'starboard',
                'next_index': int(msg.data[4]),
                'next': {'x': msg.data[5], 'y': msg.data[6]},
                'finished': msg.data[7] > 0.5,
            }

    def course_marks_callback(self, msg):
        if len(msg.data) < 4:
            return

        count = int(msg.data[0])
        expected = 4 + count * 3
        if len(msg.data) < expected:
            return

        marks = []
        offset = 4
        for index in range(count):
            marks.append({
                'index': index,
                'x': msg.data[offset],
                'y': msg.data[offset + 1],
                'rounding_side': (
                    'port' if msg.data[offset + 2] > 0.0 else 'starboard'
                ),
            })
            offset += 3

        with self.lock:
            self.state['time'] = time.time()
            self.state['course_marks'] = marks
            self.state['lap'] = int(msg.data[2])

    def autonomy_callback(self, msg):
        try:
            autonomy = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.state['time'] = time.time()
            self.state['autonomy'] = autonomy

    def buoy_callback(self, msg):
        with self.lock:
            self.state['time'] = time.time()
            self.state['buoy_relative'] = {
                'x': msg.point.x,
                'y': msg.point.y,
                'distance': math.hypot(msg.point.x, msg.point.y),
            }

    def lidar_callback(self, scan):
        points = []
        stride = max(1, len(scan.ranges) // self.max_lidar_points)
        for index in range(0, len(scan.ranges), stride):
            distance = scan.ranges[index]
            if (
                not math.isfinite(distance) or
                distance < scan.range_min or
                distance > scan.range_max
            ):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            points.append({
                'x': distance * math.cos(angle),
                'y': distance * math.sin(angle),
            })

        with self.lock:
            self.state['time'] = time.time()
            self.state['lidar_points'] = points

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SailboatDashboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
