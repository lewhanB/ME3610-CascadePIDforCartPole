"""Webots entry point. Open worlds/Cart Pole PID.wbt and press Run."""

import argparse
import csv
import math
from pathlib import Path

from control import CascadeController


def apply_target_keys(pid, current_keys, previous_keys):
    """Apply only newly pressed keys; a held key must not repeat each step."""
    pressed = current_keys - previous_keys
    if ord(" ") in pressed:
        pid.set_target(0.0)
    else:
        direction = int(ord("D") in pressed) - int(ord("A") in pressed)
        if direction:
            pid.set_target(round(pid.target + 0.10 * direction, 10))


def main():
    from controller import Keyboard, Robot

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=float, default=0.0, help="Cart target in metres (limited to +/-1.4).")
    parser.add_argument("--log", type=Path, help="Optional CSV output path; relative to this controller folder.")
    args = parser.parse_args()

    robot = Robot()
    # Read WorldInfo.basicTimeStep in milliseconds (8 ms in this world).
    timestep = int(robot.getBasicTimeStep())
    motor = robot.getDevice("cart_motor")
    position_sensor = robot.getDevice("cart_position")
    angle_sensor = robot.getDevice("pole_angle")
    if any(device is None for device in (motor, position_sensor, angle_sensor)):
        raise RuntimeError("Expected CartPole devices: cart_motor, cart_position, pole_angle.")
    # PID calculations use seconds; Webots stepping uses milliseconds.
    pid = CascadeController(timestep / 1000.0, motor.getMaxForce(), args.target)
    # Sample both joints once per simulation step.
    position_sensor.enable(timestep)
    angle_sensor.enable(timestep)
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    # setForce selects direct force control and disables Webots' motor PID.
    motor.setForce(0.0)

    log_file = None
    try:
        writer = None
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log_file = args.log.open("w", newline="")
            writer = csv.writer(log_file)
            writer.writerow([
                "time_s", "position_m", "target_m", "reference_m", "angle_rad",
                "angle_reference_rad", "velocity_m_s", "angular_velocity_rad_s",
                "force_N", "fault",
            ])
        print(f"Cascade PID ready: position loop {pid.outer_steps * timestep} ms, "
              f"balance loop {timestep} ms. "
              "A: target -0.10 m; D: target +0.10 m per press; Space: center.", flush=True)
        next_status = 0.0
        fault_reported = False
        previous_keys = set()
        # Advance Webots by one timestep and refresh sensor readings.
        # The previous force acts during this step; -1 means simulation ended.
        while robot.step(timestep) != -1:
            current_keys = set()
            key = keyboard.getKey()
            while key != -1:
                # Ignore modifier bits and normalize letter case before detecting
                # edges, so pressing Shift while holding A/D is not another press.
                code = key & Keyboard.KEY
                if ord("a") <= code <= ord("z"):
                    code -= ord("a") - ord("A")
                current_keys.add(code)
                key = keyboard.getKey()
            old_target = pid.target
            apply_target_keys(pid, current_keys, previous_keys)
            previous_keys = current_keys
            if pid.target != old_target:
                print(f"Cart target: {pid.target:+.2f} m", flush=True)

            # Read the current cart position (m) and pole angle (rad).
            position = position_sensor.getValue()
            angle = angle_sensor.getValue()
            # Compute both PID loops without advancing simulation time.
            force = pid.update(position, angle)
            # Apply the computed force (N) during the next robot.step().
            motor.setForce(force)
            # Read simulated time, not wall-clock time.
            now = robot.getTime()
            if writer:
                writer.writerow([
                    now, position, pid.target, pid.reference, angle,
                    pid.angle_reference, pid.velocity, pid.angular_velocity,
                    force, pid.fault or "",
                ])
            if pid.fault and not fault_reported:
                print(f"STOP: {pid.fault}. Force disabled; reset the simulation to restart.", flush=True)
                fault_reported = True
                if log_file:
                    log_file.flush()
            if now >= next_status:
                print(f"t={now:6.2f}s  x={position:+.4f}m  target={pid.target:+.2f}m  "
                      f"angle={math.degrees(angle):+.3f}deg  force={force:+.3f}N", flush=True)
                next_status = now + 2.0
                if log_file:
                    log_file.flush()
    finally:
        motor.setForce(0.0)
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
