#!/usr/bin/env python3

import math


def normalize_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )