#!/usr/bin/env python3

import math
from dataclasses import dataclass


@dataclass
class Vector2:

    x: float
    y: float

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

    def normalized(self):

        mag = self.magnitude()

        if mag < 1e-6:
            return Vector2(0.0, 0.0)

        return Vector2(
            self.x / mag,
            self.y / mag
        )

    def rotate(self, angle):

        return Vector2(

            self.x * math.cos(angle) -
            self.y * math.sin(angle),

            self.x * math.sin(angle) +
            self.y * math.cos(angle)

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