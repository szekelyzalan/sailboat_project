#!/usr/bin/env python3

import math
import json
import sys
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String


# =========================================================
# UTILS
# =========================================================

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def distance_between(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_vector(x, y):
    length = math.hypot(x, y)
    if length < 1e-6:
        return 1.0, 0.0
    return x / length, y / length


def interpolate_angle(start, end, ratio):
    ratio = max(0.0, min(1.0, ratio))
    return normalize_angle(start + normalize_angle(end - start) * ratio)


def strip_debug_flags(args):
    debug = False
    clean_args = []
    for arg in args:
        if arg in ('--debug', '--verbose'):
            debug = True
        else:
            clean_args.append(arg)
    return debug, clean_args


@dataclass
class CourseMark:
    name: str
    x: float
    y: float
    rounding_side: str = 'starboard'
    detected_x: float = None
    detected_y: float = None
    has_detection: bool = False

    @property
    def position(self):
        return self.x, self.y


# =========================================================
# AUTONOMY NODE
# =========================================================

class SailboatAutonomy(Node):
    MODE_SAIL_TO_MARK = 'SAIL_TO_MARK'
    MODE_OVERSHOOT_MARK = 'OVERSHOOT_MARK'
    MODE_EXIT_TURN = 'EXIT_TURN'
    LEG_DIRECT = 'DIRECT'
    LEG_TACKING = 'TACKING'

    def __init__(self, debug_default=False):
        super().__init__('sailboat_autonomy')

        self.declare_parameter('debug', debug_default)
        self.declare_parameter('verbose', debug_default)
        self.debug_enabled = self.get_parameter('debug').value
        self.verbose_enabled = self.get_parameter('verbose').value

        # STATE
        self.x = None
        self.y = None
        self.heading = 0.0
        self.apparent_wind_dir = 0.0
        self.has_apparent_wind = False
        self.detected_buoy = None

        # COURSE FALLBACK
        self.declare_parameter('mark_0_x', -510.0)
        self.declare_parameter('mark_0_y', 180.0)
        self.declare_parameter('mark_0_rounding_side', 'starboard')
        self.declare_parameter('mark_1_x', -490.0)
        self.declare_parameter('mark_1_y', 180.0)
        self.declare_parameter('mark_1_rounding_side', 'starboard')
        self.declare_parameter('mark_2_x', -490.0)
        self.declare_parameter('mark_2_y', 200.0)
        self.declare_parameter('mark_2_rounding_side', 'starboard')
        self.declare_parameter('mark_3_x', -510.0)
        self.declare_parameter('mark_3_y', 200.0)
        self.declare_parameter('mark_3_rounding_side', 'starboard')
        self.declare_parameter('use_lidar_mark_refinement', False)
        self.declare_parameter('use_lidar_close_rounding', True)
        self.declare_parameter('gps_origin_lat', 46.9216)
        self.declare_parameter('gps_origin_lon', 17.8946)
        self.declare_parameter('gps_world_origin_x', 0.0)
        self.declare_parameter('gps_world_origin_y', 0.0)
        self.marks = [
            CourseMark(
                'mark_0',
                self.get_parameter('mark_0_x').value,
                self.get_parameter('mark_0_y').value,
                self.get_parameter('mark_0_rounding_side').value,
            ),
            CourseMark(
                'mark_1',
                self.get_parameter('mark_1_x').value,
                self.get_parameter('mark_1_y').value,
                self.get_parameter('mark_1_rounding_side').value,
            ),
            CourseMark(
                'mark_2',
                self.get_parameter('mark_2_x').value,
                self.get_parameter('mark_2_y').value,
                self.get_parameter('mark_2_rounding_side').value,
            ),
            CourseMark(
                'mark_3',
                self.get_parameter('mark_3_x').value,
                self.get_parameter('mark_3_y').value,
                self.get_parameter('mark_3_rounding_side').value,
            ),
        ]
        self.use_lidar_mark_refinement = (
            self.get_parameter('use_lidar_mark_refinement').value
        )
        self.use_lidar_close_rounding = (
            self.get_parameter('use_lidar_close_rounding').value
        )
        self.origin_lat = self.get_parameter('gps_origin_lat').value
        self.origin_lon = self.get_parameter('gps_origin_lon').value
        self.world_origin_x = self.get_parameter('gps_world_origin_x').value
        self.world_origin_y = self.get_parameter('gps_world_origin_y').value
        self.current_mark = 0
        self.course_finished = False
        self.has_course_manager = False
        self.last_reported_mark = None
        self.mode = self.MODE_SAIL_TO_MARK
        self.rounding_clearance = 2.2
        self.approach_distance = 4.5
        self.turn_entry_radius = 4.5
        self.mark_capture_radius = 5.0
        self.overshoot_distance = 2.0
        self.overshoot_reached_radius = 1.2
        self.exit_distance = 2.6
        self.exit_reached_radius = 1.2
        self.mark_keepout_radius = 1.8
        self.rounding_finish_margin = 0.4
        self.overshoot_point = None
        self.exit_point = None
        self.arc_start_angle = 0.0
        self.arc_end_angle = 0.0
        self.arc_delta = 0.0
        self.outgoing_x = 1.0
        self.outgoing_y = 0.0
        self.rounding_side_x = 0.0
        self.rounding_side_y = -1.0
        self.turn_start_heading = 0.0
        self.turn_exit_heading = 0.0
        self.turn_progress = 0.0
        self.turn_progress_rate = 0.35
        self.buoy_association_radius = 12.0
        self.lidar_rounding_radius = 8.0
        self.buoy_update_gain = 0.25
        self.leg_mode = self.LEG_DIRECT
        self.tack_side = 1.0
        self.tack_timer = 0.0
        self.no_go_angle = math.radians(38.0)
        self.tack_angle = math.radians(52.0)
        self.tack_lookahead = 8.0
        self.min_tack_time = 6.0
        self.tack_switch_hysteresis = math.radians(8.0)
        self.tack_corridor_width = 6.0
        self.turn_sign = 1.0

        # RUDDER PID
        self.heading_integral = 0.0
        self.prev_heading_error = 0.0
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.4

        if self.verbose_enabled:
            self.log_course()

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
        self.create_subscription(
            PointStamped,
            '/perception/buoy/relative_position',
            self.buoy_callback,
            10
        )
        self.create_subscription(
            Float64MultiArray,
            '/course/active_leg',
            self.course_callback,
            10
        )

        # PUBLISHERS
        self.mark_rounded_pub = self.create_publisher(
            Bool,
            '/course/mark_rounded',
            10
        )
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
        self.status_pub = self.create_publisher(
            String,
            '/autonomy/status',
            10
        )

        # TIMER
        self.timer = self.create_timer(0.1, self.update)

    # =====================================================
    # CALLBACKS
    # =====================================================

    def gps_callback(self, msg):
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
        self.has_apparent_wind = True

    def buoy_callback(self, msg):
        if self.x is None:
            return

        rel_x = msg.point.x
        rel_y = msg.point.y
        world_x = (
            self.x +
            math.cos(self.heading) * rel_x -
            math.sin(self.heading) * rel_y
        )
        world_y = (
            self.y +
            math.sin(self.heading) * rel_x +
            math.cos(self.heading) * rel_y
        )
        self.detected_buoy = (world_x, world_y)
        self.refine_nearby_mark(world_x, world_y)

    def course_callback(self, msg):
        if len(msg.data) < 8:
            return

        current_index = int(msg.data[0])
        next_index = int(msg.data[4])
        needed_size = max(current_index, next_index) + 1

        while len(self.marks) < needed_size:
            mark_index = len(self.marks)
            self.marks.append(
                CourseMark(
                    f'mark_{mark_index}',
                    0.0,
                    0.0,
                    'starboard'
                )
            )

        current_mark = self.marks[current_index]
        current_mark.x = msg.data[1]
        current_mark.y = msg.data[2]
        current_mark.rounding_side = (
            'port' if msg.data[3] > 0.0 else 'starboard'
        )

        next_mark = self.marks[next_index]
        next_mark.x = msg.data[5]
        next_mark.y = msg.data[6]

        self.has_course_manager = True

        if self.current_mark != current_index:
            self.current_mark = current_index
            self.last_reported_mark = None
            current_mark.has_detection = False
            self.mode = self.MODE_SAIL_TO_MARK
            self.overshoot_point = None
            self.exit_point = None
            self.reset_leg_guidance()
            self.heading_integral = 0.0
            self.prev_heading_error = 0.0

        self.course_finished = msg.data[7] > 0.5

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def get_position(self):
        return self.x, self.y

    def log_course(self):
        for index, mark in enumerate(self.marks):
            self.get_logger().info(
                'Course mark %d: %s at (%.2f, %.2f), rounding=%s' %
                (index, mark.name, mark.x, mark.y, mark.rounding_side)
            )
        self.get_logger().info(
            'LiDAR mark refinement: %s' %
            ('enabled' if self.use_lidar_mark_refinement else 'disabled')
        )
        self.get_logger().info(
            'LiDAR close rounding: %s' %
            ('enabled' if self.use_lidar_close_rounding else 'disabled')
        )
        self.get_logger().info(
            'GPS origin: lat=%.6f lon=%.6f -> world origin (%.2f, %.2f)' %
            (
                self.origin_lat,
                self.origin_lon,
                self.world_origin_x,
                self.world_origin_y,
            )
        )

    def active_mark(self):
        return self.marks[self.current_mark]

    def next_mark(self):
        return self.marks[(self.current_mark + 1) % len(self.marks)]

    def mark_position(self, mark):
        if mark.has_detection and (
            self.use_lidar_mark_refinement or
            self.should_use_lidar_for_rounding(mark)
        ):
            return mark.detected_x, mark.detected_y
        return mark.position

    def should_use_lidar_for_rounding(self, mark):
        if (
            not self.use_lidar_close_rounding or
            not mark.has_detection or
            self.x is None or
            mark is not self.active_mark()
        ):
            return False

        expected_distance = distance_between(
            self.get_position(),
            mark.position
        )
        return (
            self.mode != self.MODE_SAIL_TO_MARK or
            expected_distance < self.lidar_rounding_radius
        )

    def reset_leg_guidance(self):
        self.leg_mode = self.LEG_DIRECT
        self.tack_side = 1.0
        self.tack_timer = 0.0

    def refine_nearby_mark(self, buoy_x, buoy_y):
        for index in (
            self.current_mark,
            (self.current_mark + 1) % len(self.marks)
        ):
            mark = self.marks[index]
            distance = math.hypot(buoy_x - mark.x, buoy_y - mark.y)
            if distance > self.buoy_association_radius:
                continue

            if mark.has_detection:
                mark.detected_x += (
                    buoy_x - mark.detected_x
                ) * self.buoy_update_gain
                mark.detected_y += (
                    buoy_y - mark.detected_y
                ) * self.buoy_update_gain
            else:
                mark.detected_x = buoy_x
                mark.detected_y = buoy_y
                mark.has_detection = True
            return

    def leg_vectors(self, mark, next_mark):
        mark_x, mark_y = self.mark_position(mark)
        next_x, next_y = self.mark_position(next_mark)
        outgoing_x = next_x - mark_x
        outgoing_y = next_y - mark_y
        outgoing_x, outgoing_y = normalize_vector(outgoing_x, outgoing_y)

        if mark.rounding_side == 'port':
            side_x = -outgoing_y
            side_y = outgoing_x
        else:
            side_x = outgoing_y
            side_y = -outgoing_x

        return outgoing_x, outgoing_y, side_x, side_y

    def approach_point(self, mark, next_mark):
        mark_x, mark_y = self.mark_position(mark)
        outgoing_x, outgoing_y, side_x, side_y = self.leg_vectors(
            mark,
            next_mark
        )

        return (
            mark_x - outgoing_x * self.approach_distance +
            side_x * self.rounding_clearance,
            mark_y - outgoing_y * self.approach_distance +
            side_y * self.rounding_clearance,
        )

    def rounding_point(self, mark, next_mark):
        mark_x, mark_y = self.mark_position(mark)
        _, _, side_x, side_y = self.leg_vectors(mark, next_mark)

        return (
            mark_x + side_x * self.rounding_clearance,
            mark_y + side_y * self.rounding_clearance,
        )

    def is_on_rounding_side(self, position, mark, next_mark):
        mark_x, mark_y = self.mark_position(mark)
        _, _, side_x, side_y = self.leg_vectors(mark, next_mark)
        side_distance = (
            (position[0] - mark_x) * side_x +
            (position[1] - mark_y) * side_y
        )
        return side_distance >= 0.0

    def keepout_clearance(self, position, mark):
        mark_x, mark_y = self.mark_position(mark)
        return distance_between(position, (mark_x, mark_y))

    def is_clear_of_mark(self, position, mark):
        return self.keepout_clearance(position, mark) >= (
            self.mark_keepout_radius + self.rounding_finish_margin
        )

    def protect_target_from_mark(self, target, mark):
        mark_x, mark_y = self.mark_position(mark)
        dx = target[0] - mark_x
        dy = target[1] - mark_y
        distance = math.hypot(dx, dy)
        minimum_radius = max(
            self.rounding_clearance,
            self.mark_keepout_radius + self.rounding_finish_margin
        )

        if distance >= minimum_radius:
            return target

        dx, dy = normalize_vector(dx, dy)
        return (
            mark_x + dx * minimum_radius,
            mark_y + dy * minimum_radius,
        )

    def start_rounding(self):
        position = self.get_position()
        mark = self.active_mark()
        next_mark = self.next_mark()
        mark_x, mark_y = self.mark_position(mark)
        next_x, next_y = self.mark_position(next_mark)
        approach_point = self.approach_point(mark, next_mark)
        turn_point = self.protect_target_from_mark(
            self.rounding_point(mark, next_mark),
            mark
        )
        _, _, side_x, side_y = self.leg_vectors(mark, next_mark)

        incoming_x = turn_point[0] - approach_point[0]
        incoming_y = turn_point[1] - approach_point[1]
        incoming_x, incoming_y = normalize_vector(incoming_x, incoming_y)

        outgoing_x = next_x - mark_x
        outgoing_y = next_y - mark_y
        outgoing_x, outgoing_y = normalize_vector(outgoing_x, outgoing_y)
        self.outgoing_x = outgoing_x
        self.outgoing_y = outgoing_y
        self.rounding_side_x = side_x
        self.rounding_side_y = side_y

        self.overshoot_point = (
            turn_point[0] + outgoing_x * self.overshoot_distance,
            turn_point[1] + outgoing_y * self.overshoot_distance,
        )
        self.exit_point = (
            turn_point[0] + outgoing_x * self.exit_distance,
            turn_point[1] + outgoing_y * self.exit_distance,
        )
        self.arc_start_angle = math.atan2(
            self.overshoot_point[1] - mark_y,
            self.overshoot_point[0] - mark_x
        )
        self.arc_end_angle = math.atan2(
            self.exit_point[1] - mark_y,
            self.exit_point[0] - mark_x
        )
        self.arc_delta = normalize_angle(
            self.arc_end_angle - self.arc_start_angle
        )
        self.turn_start_heading = math.atan2(incoming_y, incoming_x)
        self.turn_exit_heading = math.atan2(outgoing_y, outgoing_x)
        self.turn_progress = 0.0
        self.mode = self.MODE_OVERSHOOT_MARK
        self.reset_leg_guidance()

        self.heading_integral = 0.0
        self.prev_heading_error = 0.0

    def finish_rounding(self):
        rounded_index = self.current_mark
        if self.last_reported_mark != rounded_index:
            msg = Bool()
            msg.data = True
            self.mark_rounded_pub.publish(msg)
            self.last_reported_mark = rounded_index

        self.current_mark = (self.current_mark + 1) % len(self.marks)
        self.mode = self.MODE_SAIL_TO_MARK
        self.overshoot_point = None
        self.exit_point = None
        self.turn_progress = 0.0
        self.arc_start_angle = 0.0
        self.arc_end_angle = 0.0
        self.arc_delta = 0.0
        self.outgoing_x = 1.0
        self.outgoing_y = 0.0
        self.rounding_side_x = 0.0
        self.rounding_side_y = -1.0
        self.reset_leg_guidance()

        self.heading_integral = 0.0
        self.prev_heading_error = 0.0

    def apparent_wind_world(self):
        return normalize_angle(self.heading + self.apparent_wind_dir)

    def desired_heading_is_upwind(self, desired_heading):
        if not self.has_apparent_wind:
            return False

        wind_from = self.apparent_wind_world()
        wind_on_desired_heading = normalize_angle(
            wind_from - desired_heading
        )
        return abs(wind_on_desired_heading) < self.no_go_angle

    def tack_heading(self, tack_side):
        wind_from = self.apparent_wind_world()
        return normalize_angle(wind_from + tack_side * self.tack_angle)

    def choose_tack_side(self, target, desired_heading):
        target_offset = self.target_crosswind_offset(target)
        if abs(target_offset) > self.tack_corridor_width * 0.5:
            return 1.0 if target_offset > 0.0 else -1.0

        left_heading = self.tack_heading(1.0)
        right_heading = self.tack_heading(-1.0)
        left_error = abs(normalize_angle(left_heading - desired_heading))
        right_error = abs(normalize_angle(right_heading - desired_heading))
        return 1.0 if left_error < right_error else -1.0

    def target_crosswind_offset(self, target):
        if not self.has_apparent_wind or self.x is None:
            return 0.0

        dx = target[0] - self.x
        dy = target[1] - self.y
        wind_from = self.apparent_wind_world()
        cross_x = -math.sin(wind_from)
        cross_y = math.cos(wind_from)
        return dx * cross_x + dy * cross_y

    def update_tacking_side(self, target, desired_heading, dt):
        self.tack_timer += dt
        if self.tack_timer < self.min_tack_time:
            return

        crosswind_offset = self.target_crosswind_offset(target)
        crossed_layline = (
            abs(crosswind_offset) > self.tack_corridor_width and
            self.tack_side * crosswind_offset < 0.0
        )
        if crossed_layline:
            self.tack_side *= -1.0
            self.tack_timer = 0.0
            return

        current_heading = self.tack_heading(self.tack_side)
        opposite_heading = self.tack_heading(-self.tack_side)
        current_error = abs(normalize_angle(current_heading - desired_heading))
        opposite_error = abs(normalize_angle(opposite_heading - desired_heading))

        if opposite_error + self.tack_switch_hysteresis < current_error:
            self.tack_side *= -1.0
            self.tack_timer = 0.0

    def apply_leg_guidance(self, target, distance, target_kind, dt):
        if distance < self.mark_capture_radius:
            self.reset_leg_guidance()
            return target, distance, target_kind

        position = self.get_position()
        desired_heading = math.atan2(
            target[1] - position[1],
            target[0] - position[0]
        )

        if self.leg_mode == self.LEG_DIRECT:
            if not self.desired_heading_is_upwind(desired_heading):
                return target, distance, target_kind

            self.leg_mode = self.LEG_TACKING
            self.tack_side = self.choose_tack_side(target, desired_heading)
            self.tack_timer = 0.0

        self.update_tacking_side(target, desired_heading, dt)
        target_heading = self.tack_heading(self.tack_side)
        tack_target = (
            position[0] + math.cos(target_heading) * self.tack_lookahead,
            position[1] + math.sin(target_heading) * self.tack_lookahead,
        )
        return (
            tack_target,
            distance_between(position, tack_target),
            'tacking'
        )

    def compute_guidance_target(self, dt):
        position = self.get_position()
        mark = self.active_mark()

        if self.mode == self.MODE_SAIL_TO_MARK:
            next_mark = self.next_mark()
            approach_point = self.approach_point(mark, next_mark)
            mark_distance = distance_between(
                position,
                self.mark_position(mark)
            )
            approach_point_distance = distance_between(position, approach_point)
            if (
                (
                    mark_distance < self.mark_capture_radius and
                    self.is_on_rounding_side(position, mark, next_mark)
                ) or
                approach_point_distance < self.turn_entry_radius
            ):
                self.start_rounding()
            else:
                return self.apply_leg_guidance(
                    approach_point,
                    approach_point_distance,
                    'approach_point',
                    dt
                )

        if self.mode == self.MODE_OVERSHOOT_MARK:
            overshoot_distance = distance_between(position, self.overshoot_point)
            if overshoot_distance < self.overshoot_reached_radius:
                self.mode = self.MODE_EXIT_TURN
            else:
                return self.overshoot_point, overshoot_distance, 'overshoot'

        if self.mode == self.MODE_EXIT_TURN:
            mark_x, mark_y = self.mark_position(mark)
            boat_arc_angle = math.atan2(
                position[1] - mark_y,
                position[0] - mark_x
            )
            outgoing_progress = (
                (position[0] - mark_x) * self.outgoing_x +
                (position[1] - mark_y) * self.outgoing_y
            )
            if abs(self.arc_delta) < 1e-6:
                actual_arc_progress = 1.0
            else:
                actual_arc_progress = (
                    normalize_angle(
                        boat_arc_angle - self.arc_start_angle
                    ) / self.arc_delta
                )
            actual_arc_progress = max(
                0.0,
                min(1.0, actual_arc_progress)
            )

            self.turn_progress = min(
                1.0,
                self.turn_progress + self.turn_progress_rate * dt
            )
            arc_angle = self.arc_start_angle + (
                self.arc_delta * self.turn_progress
            )
            radius = max(
                self.rounding_clearance,
                self.mark_keepout_radius + self.rounding_finish_margin
            )
            arc_target = (
                mark_x + math.cos(arc_angle) * radius,
                mark_y + math.sin(arc_angle) * radius,
            )
            arc_target = self.protect_target_from_mark(arc_target, mark)

            rounded_mark = (
                outgoing_progress >= self.exit_distance and
                self.is_clear_of_mark(position, mark) and
                self.is_on_rounding_side(position, mark, self.next_mark()) and
                (
                    actual_arc_progress >= 0.7 or
                    self.turn_progress >= 0.9
                )
            )
            if rounded_mark:
                self.finish_rounding()
                next_mark = self.active_mark()
                next_turn_point = self.rounding_point(
                    next_mark,
                    self.next_mark()
                )
                return (
                    next_turn_point,
                    distance_between(position, next_turn_point),
                    'next_rounding_point'
                )
            if self.turn_progress >= 1.0:
                continue_distance = max(
                    outgoing_progress + 2.0,
                    self.exit_distance + 2.0
                )
                continue_target = (
                    mark_x + self.outgoing_x * continue_distance +
                    self.rounding_side_x * self.rounding_clearance,
                    mark_y + self.outgoing_y * continue_distance +
                    self.rounding_side_y * self.rounding_clearance,
                )
                continue_target = self.protect_target_from_mark(
                    continue_target,
                    mark
                )
                return (
                    continue_target,
                    distance_between(position, continue_target),
                    'continue_rounding'
                )
            return arc_target, distance_between(position, arc_target), 'arc'

        approach_point = self.approach_point(mark, self.next_mark())
        return self.apply_leg_guidance(
            approach_point,
            distance_between(position, approach_point),
            'approach_point',
            dt
        )

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
        # TARGET FROM GUIDANCE STATE MACHINE
        target, distance, target_kind = self.compute_guidance_target(dt)
        tx, ty = target
        dx = tx - self.x
        dy = ty - self.y
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
        self.publish_status(target, distance, target_kind, heading_error)
        self.debug_print_status(
            target_kind,
            target,
            distance,
            heading_error,
            rudder,
            sail_angle
        )

    def publish_status(self, target, distance, target_kind, heading_error):
        mark = self.active_mark()
        mark_x, mark_y = self.mark_position(mark)
        msg = String()
        msg.data = json.dumps({
            'mode': self.mode,
            'leg_mode': self.leg_mode,
            'tack_side': self.tack_side,
            'target_kind': target_kind,
            'target_x': target[0],
            'target_y': target[1],
            'target_distance': distance,
            'heading_error_deg': math.degrees(heading_error),
            'active_mark': self.current_mark,
            'active_mark_name': mark.name,
            'active_mark_x': mark_x,
            'active_mark_y': mark_y,
            'mark_distance': distance_between(
                self.get_position(),
                (mark_x, mark_y)
            ),
            'mark_keepout_radius': self.mark_keepout_radius,
            'rounding_clearance': self.rounding_clearance,
            'rounding_side_ok': self.is_on_rounding_side(
                self.get_position(),
                mark,
                self.next_mark()
            ),
            'lidar_mark_active': self.should_use_lidar_for_rounding(mark),
        })
        self.status_pub.publish(msg)

    def debug_print_status(
        self,
        target_kind,
        target,
        distance,
        heading_error,
        rudder,
        sail_angle
    ):
        if not self.debug_enabled:
            return

        print("\n====================")
        print("MODE      :", self.mode)
        print("LEG MODE  :", self.leg_mode)
        print("TACK SIDE :", round(self.tack_side, 1))
        print("TARGET    :", target_kind)
        print("MARK      :", self.active_mark().name)
        print("BOAT XY   :", round(self.x, 2), round(self.y, 2))
        print(
            "MARK XY   :",
            round(self.mark_position(self.active_mark())[0], 2),
            round(self.mark_position(self.active_mark())[1], 2)
        )
        print(
            "MARK DIST :",
            round(
                distance_between(
                    self.get_position(),
                    self.mark_position(self.active_mark())
                ),
                2
            )
        )
        print(
            "ROUND SIDE:",
            self.is_on_rounding_side(
                self.get_position(),
                self.active_mark(),
                self.next_mark()
            )
        )
        print("DISTANCE  :", round(distance, 2))
        print("HEADING ERR:", round(math.degrees(heading_error), 2))
        print("RUDDER:", round(rudder, 2))
        print("SAIL:", round(math.degrees(sail_angle), 2))


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    raw_args = sys.argv[1:] if args is None else args
    debug_enabled, clean_args = strip_debug_flags(raw_args)
    rclpy.init(args=clean_args)
    node = SailboatAutonomy(debug_default=debug_enabled)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
