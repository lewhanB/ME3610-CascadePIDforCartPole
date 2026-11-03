"""Offline regression checks; run with python3 -m unittest discover -s tests.

The plant is an independent nonlinear planar model of the supplied PROTO,
integrated with RK4 under a zero-order-held motor force. It uses the uniform
box's hinge inertia and both joint damping constants. Contact, joint stops,
and Webots' solver are excluded, so these checks complement simulator tests.
"""

import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "cascade_pid"))
from control import CascadeController, PID


DT = 0.008


def acceleration(state, force):
    """Solve the two coupled rigid-body equations, positive angle toward +X."""
    _, velocity, angle, angular_velocity = state
    total_mass = 1.0 + 0.20
    mass_length = 0.20 * 0.30
    # I_com = m * (length^2 + thickness^2) / 12, then parallel axis.
    hinge_inertia = 0.20 * (0.60**2 + 0.03**2) / 12 + 0.20 * 0.30**2
    coupling = mass_length * math.cos(angle)
    horizontal = force - 0.05 * velocity + mass_length * math.sin(angle) * angular_velocity**2
    angular = mass_length * 9.81 * math.sin(angle) - 0.001 * angular_velocity
    determinant = total_mass * hinge_inertia - coupling**2
    return (
        velocity,
        (hinge_inertia * horizontal - coupling * angular) / determinant,
        angular_velocity,
        (total_mass * angular - coupling * horizontal) / determinant,
    )


def plant_step(state, force):
    def advance(derivative, scale):
        return tuple(value + DT * scale * rate for value, rate in zip(state, derivative))

    k1 = acceleration(state, force)
    k2 = acceleration(advance(k1, 0.5), force)
    k3 = acceleration(advance(k2, 0.5), force)
    k4 = acceleration(advance(k3, 1.0), force)
    return tuple(
        value + DT * (a + 2 * b + 2 * c + d) / 6
        for value, a, b, c, d in zip(state, k1, k2, k3, k4)
    )


class NonlinearBalanceTests(unittest.TestCase):
    def simulate(self, initial_angle=0.05, initial_position=0.0, target=0.0, push=False):
        pid = CascadeController(DT)
        state = (initial_position, 0.0, initial_angle, 0.0)
        maximum_position = abs(initial_position)
        maximum_force = 0.0
        tail = []
        for step in range(round(30.0 / DT)):
            time = step * DT
            if time >= 2.0:
                pid.set_target(target)
            force = pid.update(state[0], state[2])
            self.assertIsNone(pid.fault, f"Fault at {time:.3f}s: {pid.fault}")
            self.assertLessEqual(abs(force), 20.0)
            self.assertLessEqual(abs(pid.angle_reference), 0.12)
            disturbance = 5.0 if push and 5.0 <= time < 5.2 else 0.0
            state = plant_step(state, force + disturbance)
            maximum_position = max(maximum_position, abs(state[0]))
            maximum_force = max(maximum_force, abs(force))
            if time >= 28.0:
                tail.append(state)

        # Check the whole last two seconds so a zero crossing cannot pass.
        self.assertLess(max(abs(s[0] - target) for s in tail), 0.003)
        self.assertLess(max(abs(s[2]) for s in tail), 0.001)
        self.assertLess(max(abs(s[1]) for s in tail), 0.003)
        self.assertLess(max(abs(s[3]) for s in tail), 0.003)
        self.assertLess(maximum_position, CascadeController.TRAVEL_LIMIT)
        return maximum_position, maximum_force

    def test_recovers_initial_tilts(self):
        for angle in (-0.15, -0.05, 0.05, 0.15, 0.30):
            with self.subTest(initial_angle=angle):
                self.simulate(initial_angle=angle)

    def test_returns_displaced_cart_to_center(self):
        self.simulate(initial_angle=0.0, initial_position=0.5)

    def test_tracks_positive_and_negative_targets(self):
        for target in (-0.5, 0.5):
            with self.subTest(target=target):
                self.simulate(target=target)

    def test_rejects_external_force_pulse(self):
        self.simulate(push=True)

    def test_default_recovery_uses_small_cart_excursion(self):
        maximum_position, maximum_force = self.simulate()
        self.assertLess(maximum_position, 0.08)
        self.assertLess(maximum_force, 8.0)


class ControllerLimitsTests(unittest.TestCase):
    def test_integral_does_not_wind_up_against_saturation(self):
        pid = PID(kp=2.0, ki=1.0, kd=0.0, output_limit=1.0, integral_limit=4.0)
        for _ in range(1000):
            self.assertEqual(pid.update(error=1.0, error_rate=0.0, dt=DT), 1.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertTrue(pid.saturated)
        self.assertEqual(pid.update(error=0.0, error_rate=0.0, dt=DT), 0.0)

    def test_integral_unwinds_and_can_be_frozen(self):
        pid = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=1.0, integral_limit=4.0, integral=2.0)
        pid.update(error=-0.1, error_rate=0.0, dt=1.0)
        self.assertAlmostEqual(pid.integral, 1.9)
        pid.update(error=-0.1, error_rate=0.0, dt=1.0, integrate=False)
        self.assertAlmostEqual(pid.integral, 1.9)

    def test_faults_latch_and_disable_force(self):
        cases = (
            (math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0),
            (0.0, -math.inf), (1.9, 0.0), (-1.9, 0.0),
            (0.0, 0.6), (0.0, -0.6),
        )
        for position, angle in cases:
            with self.subTest(position=position, angle=angle):
                pid = CascadeController()
                self.assertEqual(pid.update(position, angle), 0.0)
                self.assertIsNotNone(pid.fault)
                fault = pid.fault
                self.assertEqual(pid.update(0.0, 0.05), 0.0)
                self.assertEqual(pid.fault, fault)

    def test_target_limits_and_reference_slew(self):
        pid = CascadeController(target=10.0)
        self.assertEqual(pid.target, 1.4)
        for _ in range(20):
            previous = pid.reference
            pid.update(0.0, 0.0)
            self.assertLessEqual(abs(pid.reference - previous), pid.TARGET_SPEED * DT + 1e-12)
        pid.set_target(-10.0)
        self.assertEqual(pid.target, -1.4)
        with self.assertRaises(ValueError):
            pid.set_target(math.nan)

    def test_respects_motor_force_rating(self):
        pid = CascadeController(force_limit=2.0)
        self.assertEqual(pid.update(0.0, 0.3), 2.0)

    def test_rejects_invalid_configuration(self):
        for timestep in (0.0, -0.008, 0.032, math.nan, math.inf):
            with self.subTest(timestep=timestep), self.assertRaises(ValueError):
                CascadeController(dt=timestep)
        for force in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(force=force), self.assertRaises(ValueError):
                CascadeController(force_limit=force)


if __name__ == "__main__":
    unittest.main()
