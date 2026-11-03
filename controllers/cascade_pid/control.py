"""Cascade PID for the supplied CartPole: SI units, no Webots dependency."""

import math
from dataclasses import dataclass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float  # Limit on the integral's output contribution.
    integral: float = 0.0
    saturated: bool = False

    def update(self, error, error_rate, dt, integrate=True):
        """Derivative on measurement; conditional integration prevents windup."""
        candidate = self.integral
        if integrate and self.ki:
            # Accumulate Ki * error * dt, with an integral contribution limit.
            candidate = clamp(
                self.integral + self.ki * error * dt,
                -self.integral_limit, self.integral_limit,
            )
        # PID output = proportional + integral + measured-motion derivative.
        raw = self.kp * error + candidate + self.kd * error_rate
        # Allow integration only when unsaturated or unwinding saturation.
        if abs(raw) <= self.output_limit or raw * error < 0.0:
            self.integral = candidate
        raw = self.kp * error + self.integral + self.kd * error_rate
        self.saturated = abs(raw) > self.output_limit
        return clamp(raw, -self.output_limit, self.output_limit)


class CascadeController:
    """Position PID -> desired pole angle -> reverse-acting angle PID -> force.

    Positive pole angle leans toward +X. Positive force moves the cart +X,
    reducing that angle. Consequently the inner error is angle - reference.
    """

    TARGET_LIMIT = 1.4  # m; leave room before the +/-2 m physical stops.
    TRAVEL_LIMIT = 1.9  # m
    FALL_LIMIT = 0.6  # rad; this controller balances, it does not swing up.
    TARGET_SPEED = 0.25  # m/s, reference slew limit
    DERIVATIVE_TAU = 0.02  # s, first-order velocity filter

    def __init__(self, dt=0.008, force_limit=20.0, target=0.0):
        if not math.isfinite(dt) or not 0 < dt <= 0.016:
            raise ValueError("Use a Webots basicTimeStep of 16 ms or less (8 ms recommended).")
        if not math.isfinite(force_limit) or force_limit <= 0:
            raise ValueError("Motor force limit must be positive and finite.")
        self.dt = dt
        # Outer loop runs every four inner steps at the default 8 ms timestep.
        self.outer_steps = max(1, round(0.032 / dt))
        # Gains: Kp, Ki, Kd, output limit, integral contribution limit.
        self.position_pid = PID(0.10, 0.015, 0.16, 0.12, 0.03)
        self.angle_pid = PID(100.0, 5.0, 18.0, min(force_limit, 20.0), 3.0)
        self.target = 0.0
        self.set_target(target)
        self.reference = 0.0
        self.angle_reference = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.previous_position = None
        self.previous_angle = None
        self.steps = 0
        self.fault = None
        self.inner_saturated = False

    def set_target(self, target):
        if not math.isfinite(target):
            raise ValueError("Target must be finite.")
        self.target = clamp(target, -self.TARGET_LIMIT, self.TARGET_LIMIT)

    def update(self, position, angle):
        # Called once per Webots step; this function only calculates force.
        # Faults latch the force to zero until the controller is restarted.
        if self.fault:
            return 0.0
        if not math.isfinite(position) or not math.isfinite(angle):
            self.fault = "Invalid sensor reading"
        elif abs(position) >= self.TRAVEL_LIMIT:
            self.fault = "Cart reached the travel guard"
        elif abs(angle) >= self.FALL_LIMIT:
            self.fault = "Pole is outside the balancing range"
        if self.fault:
            return 0.0

        if self.previous_position is not None:
            # Estimate velocities from sample differences, then low-pass filter.
            alpha = self.dt / (self.DERIVATIVE_TAU + self.dt)
            self.velocity += alpha * (
                (position - self.previous_position) / self.dt - self.velocity
            )
            self.angular_velocity += alpha * (
                (angle - self.previous_angle) / self.dt - self.angular_velocity
            )
        self.previous_position = position
        self.previous_angle = angle

        # Ramp the position reference to avoid sudden target jumps.
        self.reference += clamp(
            self.target - self.reference,
            -self.TARGET_SPEED * self.dt, self.TARGET_SPEED * self.dt,
        )
        if self.steps % self.outer_steps == 0:
            # Outer PID: position error -> desired pole angle (held between updates).
            self.angle_reference = self.position_pid.update(
                self.reference - position, -self.velocity,
                self.outer_steps * self.dt,
                integrate=not self.inner_saturated,  # Pause if motor force saturated.
            )
            self.inner_saturated = False

        # Inner PID: pole angle error -> cart force, updated every step.
        # Positive lean needs positive force, so error is measured minus desired.
        force = self.angle_pid.update(
            angle - self.angle_reference, self.angular_velocity, self.dt,
        )
        # Remember any force saturation until the next outer-loop update.
        self.inner_saturated |= self.angle_pid.saturated
        # Count controller updates; only robot.step() advances Webots time.
        self.steps += 1
        return force
