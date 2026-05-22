#!/usr/bin/env python3

import math

from sailboat_physics.vector2 import Vector2
from sailboat_physics.utils import normalize_angle


def compute_true_wind(
    wind_speed,
    wind_direction
):

    return Vector2(

        wind_speed *
        math.cos(wind_direction),

        wind_speed *
        math.sin(wind_direction)

    )


def compute_apparent_wind(
    true_wind,
    boat_velocity
):

    return (
        true_wind -
        boat_velocity
    )


def compute_angle_of_attack(
    app_wind_direction,
    yaw,
    sail_angle
):

    aoa = (
        app_wind_direction -
        yaw -
        sail_angle
    )

    return normalize_angle(aoa)


def compute_sail_force(
    apparent_wind,
    angle_of_attack
):

    app_wind_speed = apparent_wind.magnitude()
    app_wind_direction = apparent_wind.direction()

    # COEFFICIENTS

    lift_coefficient = math.sin(
        2.0 * angle_of_attack
    )

    drag_coefficient = (
        1.0 -
        math.cos(angle_of_attack)
    )

    # FORCE MAGNITUDES

    lift_gain = 0.8
    drag_gain = 0.3

    lift_force = (
        app_wind_speed ** 2 *
        lift_coefficient *
        lift_gain
    )

    drag_force = (
        app_wind_speed ** 2 *
        drag_coefficient *
        drag_gain
    )

    # DRAG DIRECTION

    drag_direction = Vector2(
        math.cos(app_wind_direction),
        math.sin(app_wind_direction)
    )

    # LIFT DIRECTION

    lift_sign = 1.0

    if angle_of_attack < 0.0:
        lift_sign = -1.0

    lift_direction_angle = (
        app_wind_direction +
        lift_sign * math.pi / 2.0
    )

    lift_direction = Vector2(
        math.cos(lift_direction_angle),
        math.sin(lift_direction_angle)
    )

    # TOTAL FORCE

    force = (
        lift_direction * lift_force +
        drag_direction * drag_force
    )

    return force


def apply_water_drag(
    velocity
):

    water_drag = 0.98

    return velocity * water_drag


def apply_keel_damping(
    velocity,
    yaw,
    dt
):

    side_direction = Vector2(
        -math.sin(yaw),
        math.cos(yaw)
    )

    side_velocity = velocity.dot(
        side_direction
    )

    keel_strength = 0.9

    damping = (
        side_direction *
        side_velocity *
        keel_strength *
        dt
    )

    return (
        velocity -
        damping
    )


def compute_forward_speed(
    velocity,
    yaw
):

    forward_direction = Vector2(
        math.cos(yaw),
        math.sin(yaw)
    )

    return velocity.dot(
        forward_direction
    )


def compute_rudder_yaw_rate(
    rudder_angle,
    forward_speed
):

    rudder_gain = 0.5

    return (
        rudder_angle *
        rudder_gain *
        forward_speed
    )