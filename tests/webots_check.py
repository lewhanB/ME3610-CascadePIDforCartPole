"""Check one cart-pole at a time, leaving the working world intact.

Usage: python3 tests/webots_check.py [--webots /path/to/webots]
No external Python packages required. Webots must be installed.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = [
    ("default", 0.05, 0.0),
    ("negative_tilt", -0.05, 0.0),
    ("larger_tilt", 0.15, 0.0),
    ("larger_negative_tilt", -0.15, 0.0),
    ("positive_target", 0.05, 0.5),
    ("negative_target", -0.05, -0.5),
    ("push", 0.05, 0.0),
]

SUPERVISOR = '''import os
import sys
from controller import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
cart = robot.getFromDef("CART_POLE").getFromProtoDef("CART")
automated = os.environ.get("CARTPOLE_AUTOMATED_TEST") == "1"
while robot.step(step) != -1:
    t = robot.getTime()
    if "push" in sys.argv[1:] and 5.0 <= t < 5.2:
        cart.addForce([5.0, 0.0, 0.0], False)
    # Only the test runner requests an automatic exit. Reopening this world
    # manually must not unexpectedly close the Webots window.
    if automated and t >= 30.0:
        robot.simulationQuit(0)
        break
'''


def prepare(destination, scenario):
    (destination / "worlds").mkdir(parents=True)
    (destination / "protos").mkdir()
    shutil.copy2(ROOT / "protos/CartPole.proto", destination / "protos/CartPole.proto")
    shutil.copytree(ROOT / "controllers/cascade_pid", destination / "controllers/cascade_pid",
                    ignore=shutil.ignore_patterns("__pycache__", "*.csv"))
    supervisor = destination / "controllers/check_supervisor"
    supervisor.mkdir()
    (supervisor / "check_supervisor.py").write_text(SUPERVISOR)
    world = (ROOT / "worlds/Cart Pole PID.wbt").read_text()
    # Retain the supplied environment with exactly one cart-pole.
    world = world[:world.index("DEF CART_POLE CartPole")]
    name, angle, target = scenario
    world += f'''\nDEF CART_POLE CartPole {{
  name "{name}"
  initialAngle {angle}
  controllerArgs [ "--target" "{target}" "--log" "{destination / (name + '.csv')}" ]
}}
'''
    world += ('\nRobot { supervisor TRUE controller "check_supervisor" '
              f'controllerArgs [ "{name}" ] }}\n')
    path = destination / "worlds/check.wbt"
    path.write_text(world)
    return path


def check_results(destination, scenarios):
    summary = {}
    for name, initial_angle, target in scenarios:
        with (destination / (name + ".csv")).open() as stream:
            rows = list(csv.DictReader(stream))
        assert rows and float(rows[-1]["time_s"]) >= 29.9, f"{name}: incomplete run"
        assert not any(row["fault"] for row in rows), f"{name}: controller guard tripped"
        # Check actual hinge initialization, not just convergence to a wrong zero.
        assert abs(float(rows[0]["angle_rad"]) - initial_angle) < 0.002, name
        numeric = ("position_m", "angle_rad", "force_N")
        assert all(math.isfinite(float(row[key])) for row in rows for key in numeric), name
        max_position = max(abs(float(row["position_m"])) for row in rows)
        max_force = max(abs(float(row["force_N"])) for row in rows)
        tail = [row for row in rows if float(row["time_s"]) >= 25]
        position_error = max(abs(float(row["position_m"]) - target) for row in tail)
        angle_error = max(abs(float(row["angle_rad"])) for row in tail)
        assert max_position < 1.9, f"{name}: excessive travel {max_position}"
        assert max_force <= 20.000001, f"{name}: excessive force {max_force}"
        assert position_error < 0.01, f"{name}: unsettled position {position_error}"
        assert angle_error < 0.002, f"{name}: unsettled angle {angle_error}"
        summary[name] = {
            "peak_abs_position_m": round(max_position, 6),
            "peak_abs_force_N": round(max_force, 6),
            "final_5s_max_position_error_m": round(position_error, 6),
            "final_5s_max_angle_rad": round(angle_error, 6),
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", default=shutil.which("webots") or
                        "/Applications/Webots.app/Contents/MacOS/webots")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--scenario", choices=[s[0] for s in SCENARIOS] + ["all"],
                        default="default", help="Default: one cart-pole balance check. 'all' runs sequentially.")
    args = parser.parse_args()
    destination = Path(tempfile.mkdtemp(prefix="cartpole-webots-"))
    print(f"Test project and logs: {destination}", flush=True)
    scenarios = [s for s in SCENARIOS if args.scenario in (s[0], "all")]
    summary = {}
    for scenario in scenarios:
        project = destination / scenario[0]
        world = prepare(project, scenario)
        print(f"Running {scenario[0]} with one cart-pole; this automated test "
              "closes its Webots window after 30 simulated seconds.", flush=True)
        command = [args.webots, "--batch", "--mode=fast", "--no-rendering", "--minimize",
                   "--stdout", "--stderr", str(world)]
        try:
            completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, timeout=args.timeout,
                                       env={**os.environ, "CARTPOLE_AUTOMATED_TEST": "1"})
        except subprocess.TimeoutExpired as error:
            output = error.stdout or b""
            (project / "webots.log").write_bytes(output if isinstance(output, bytes) else output.encode())
            raise SystemExit(f"Webots timed out. Inspect {project / 'webots.log'}")
        (project / "webots.log").write_text(completed.stdout)
        if completed.returncode:
            raise SystemExit(f"Webots exited {completed.returncode}. Inspect {project / 'webots.log'}")
        summary.update(check_results(project, [scenario]))
    (destination / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"PASS: {len(scenarios)} scenario(s), one cart-pole per world, 30 simulated seconds each.")


if __name__ == "__main__":
    main()
