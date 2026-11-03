"""Keyboard press behavior and automatic-test lifecycle checks, without a GUI."""

import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers/cascade_pid"))
from cascade_pid import apply_target_keys
from control import CascadeController
from webots_check import SUPERVISOR


class TargetKeysTests(unittest.TestCase):
    def test_hold_release_and_press_again(self):
        pid = CascadeController()
        previous = set()
        for keys in ({ord("D")},) * 100:
            apply_target_keys(pid, keys, previous)
            previous = keys
        self.assertEqual(pid.target, 0.10)
        apply_target_keys(pid, set(), previous)
        apply_target_keys(pid, {ord("D")}, set())
        self.assertEqual(pid.target, 0.20)
        apply_target_keys(pid, {ord("A")}, set())
        self.assertEqual(pid.target, 0.10)

    def test_center_opposing_keys_and_limits(self):
        pid = CascadeController()
        apply_target_keys(pid, {ord("A"), ord("D")}, set())
        self.assertEqual(pid.target, 0.0)
        for _ in range(30):
            apply_target_keys(pid, {ord("D")}, set())
        self.assertEqual(pid.target, 1.4)
        apply_target_keys(pid, {ord(" ")}, set())
        self.assertEqual(pid.target, 0.0)
        for _ in range(30):
            apply_target_keys(pid, {ord("A")}, set())
        self.assertEqual(pid.target, -1.4)


class TestWorldLifecycleTests(unittest.TestCase):
    def run_supervisor(self, automated):
        times = iter((0.008, 30.0, 60.0))
        state = {"time": 0.0, "quit": None, "steps": 0}

        def step(_):
            try:
                state["time"] = next(times)
                state["steps"] += 1
                return 0
            except StopIteration:
                return -1

        def quit_simulation(code):
            state["quit"] = code

        robot = SimpleNamespace(
            getBasicTimeStep=lambda: 8,
            getFromDef=lambda _: SimpleNamespace(getFromProtoDef=lambda _: None),
            step=step, getTime=lambda: state["time"], simulationQuit=quit_simulation,
        )
        with patch.dict(os.environ, {"CARTPOLE_AUTOMATED_TEST": "1" if automated else ""}), \
                patch.dict(sys.modules, {"controller": SimpleNamespace(Supervisor=lambda: robot)}), \
                patch.object(sys, "argv", ["check_supervisor.py", "default"]):
            exec(compile(SUPERVISOR, "check_supervisor.py", "exec"), {})
        return state

    def test_manual_world_keeps_running_past_thirty_seconds(self):
        state = self.run_supervisor(False)
        self.assertIsNone(state["quit"])
        self.assertEqual(state["time"], 60.0)

    def test_automated_runner_still_exits_at_thirty_seconds(self):
        state = self.run_supervisor(True)
        self.assertEqual(state["quit"], 0)
        self.assertEqual(state["time"], 30.0)


if __name__ == "__main__":
    unittest.main()
