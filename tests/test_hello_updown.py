import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


motors = types.ModuleType("lerobot.motors")
motors.Motor = lambda *args: args
motors.MotorNormMode = types.SimpleNamespace(DEGREES=1, RANGE_0_100=2)
feetech = types.ModuleType("lerobot.motors.feetech")
feetech.FeetechMotorsBus = object
spec = importlib.util.spec_from_file_location(
    "hello_updown", Path(__file__).resolve().parents[1] / "hello_updown.py"
)
wave = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"lerobot": types.ModuleType("lerobot"),
                             "lerobot.motors": motors,
                             "lerobot.motors.feetech": feetech}):
    spec.loader.exec_module(wave)


class FakeBus:
    def __init__(self, fail_enable=None):
        self.is_connected = False
        self.positions = {joint: 2000 + index * 10 for index, joint in enumerate(wave.MOTORS)}
        # Exercise degenerate limits on stationary and moving joints.
        self.limits = {joint: [2046, 2046] for joint in wave.MOTORS}
        self.goals = {}
        self.enabled = set()
        self.events = []
        self.fail_enable = fail_enable

    def connect(self, handshake=False):
        self.is_connected = True

    def read(self, register, joint, **kwargs):
        if register == "Present_Position":
            return self.positions[joint]
        return self.limits[joint][register == "Max_Position_Limit"]

    def write(self, register, joint, value, **kwargs):
        self.events.append((register, joint, value))
        if register == "Goal_Position":
            lo, hi = self.limits[joint]
            assert lo <= value <= hi
            self.goals[joint] = value
        else:
            assert joint not in self.enabled
            self.limits[joint][register == "Max_Position_Limit"] = value

    def enable_torque(self, joint, **kwargs):
        assert self.goals[joint] == self.positions[joint]
        if joint == self.fail_enable:
            raise RuntimeError("simulated enable failure")
        self.enabled.add(joint)

    def disable_torque(self, joint, **kwargs):
        self.events.append(("disable", joint))
        self.enabled.discard(joint)

    def disconnect(self, disable_torque=True):
        self.disconnected_with = disable_torque
        self.is_connected = False


class HoldingTests(unittest.TestCase):
    def run_wave(self, bus, dry=False, interrupt=False):
        def offsets(*args):
            if not dry:
                self.assertEqual(bus.enabled, set(wave.MOTORS))
            yield 10
            if interrupt:
                raise KeyboardInterrupt
            yield 0

        argv = ["hello_updown.py", "--port", "fake"] + (["--dry-run"] if dry else [])
        with patch.object(wave, "FeetechMotorsBus", return_value=bus), \
             patch.object(wave, "wave_offsets", offsets), \
             patch.object(wave.time, "sleep"), patch.object(sys, "argv", argv), \
             contextlib.redirect_stdout(io.StringIO()):
            wave.main()

    def assert_clean(self, bus):
        self.assertFalse(bus.enabled)
        self.assertFalse(bus.is_connected)
        self.assertTrue(all(limits == [2046, 2046] for limits in bus.limits.values()))

    def test_other_servos_keep_initial_goals(self):
        bus = FakeBus()
        self.run_wave(bus)
        for joint, position in bus.positions.items():
            goals = [e[2] for e in bus.events if e[:2] == ("Goal_Position", joint)]
            if joint != wave.JOINT:
                self.assertEqual(goals, [position, position])
            else:
                self.assertIn(position + 10, goals)
                self.assertEqual(goals[-1], position)
        self.assert_clean(bus)

    def test_each_run_adopts_its_starting_pose_despite_stale_goals(self):
        bus = FakeBus()
        for base in (900, 2800):
            with self.subTest(base=base):
                bus.positions = {joint: base + index * 20
                                 for index, joint in enumerate(wave.MOTORS)}
                # The second run retains goals from the previous run until
                # main replaces them with newly measured positions.
                bus.events.clear()
                self.run_wave(bus)
                for joint, position in bus.positions.items():
                    goals = [e[2] for e in bus.events
                             if e[:2] == ("Goal_Position", joint)]
                    self.assertEqual(goals[0], position)
                    self.assertEqual(goals[-1], position)
                    if joint == wave.JOINT:
                        self.assertIn(position + 10, goals)
                    else:
                        self.assertTrue(all(goal == position for goal in goals))
                self.assert_clean(bus)

    def test_partial_enable_failure_releases_and_restores_all(self):
        bus = FakeBus(fail_enable="elbow_flex")
        with self.assertRaisesRegex(RuntimeError, "simulated"):
            self.run_wave(bus)
        self.assert_clean(bus)

    def test_interrupt_releases_and_restores_all(self):
        bus = FakeBus()
        self.run_wave(bus, interrupt=True)
        self.assert_clean(bus)

    def test_dry_run_never_writes_or_changes_torque(self):
        bus = FakeBus()
        self.run_wave(bus, dry=True)
        self.assertEqual(bus.events, [])
        self.assertFalse(bus.disconnected_with)


if __name__ == "__main__":
    unittest.main()
