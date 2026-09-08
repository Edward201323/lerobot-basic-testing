import contextlib
import importlib.util
import io
import math
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
    "arm_wave", Path(__file__).resolve().parents[1] / "jerkoff.py"
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
        self.limits[wave.GRIPPER] = [0, 4095]
        self.original_limits = {joint: list(limits) for joint, limits in self.limits.items()}
        self.goals = {}
        self.enabled = set()
        self.events = []
        self.fail_enable = fail_enable
        self.telemetry = {"Status": 0, "Present_Load": 100, "Present_Temperature": 30,
                          "Present_Voltage": 74, "Present_Current": 50}

    def connect(self, handshake=False):
        self.is_connected = True

    def read(self, register, joint, **kwargs):
        if register == "Present_Position":
            return self.positions[joint]
        if register == "Goal_Position":
            return self.goals[joint]
        if register == "Torque_Enable":
            return int(joint in self.enabled)
        if register in ("Min_Position_Limit", "Max_Position_Limit"):
            return self.limits[joint][register == "Max_Position_Limit"]
        return self.telemetry[register]

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
    def run_wave(self, bus, dry=False, interrupt=False, diagnostics=False,
                 extra_args=(), samples=(10, 0)):
        def offsets(*args):
            self.motion_parameters = args
            if not dry:
                self.assertEqual(bus.enabled, set(wave.MOTORS))
            yield samples[0]
            if interrupt:
                raise KeyboardInterrupt
            yield from samples[1:]

        argv = ["jerkoff.py", "--port", "fake"] + (["--dry-run"] if dry else [])
        if diagnostics:
            argv.append("--diagnostics")
        argv.extend(extra_args)
        with patch.object(wave, "FeetechMotorsBus", return_value=bus), \
             patch.object(wave, "wave_offsets", offsets), \
             patch.object(wave.time, "sleep"), patch.object(sys, "argv", argv), \
             contextlib.redirect_stdout(io.StringIO()):
            wave.main()

    def assert_clean(self, bus):
        self.assertFalse(bus.enabled)
        self.assertFalse(bus.is_connected)
        self.assertEqual(bus.limits, bus.original_limits)

    def test_other_servos_keep_initial_goals(self):
        bus = FakeBus()
        self.run_wave(bus)
        for joint, position in bus.positions.items():
            goals = [e[2] for e in bus.events if e[:2] == ("Goal_Position", joint)]
            if joint not in (wave.JOINT, wave.GRIPPER):
                self.assertEqual(goals, [position, position])
            elif joint == wave.JOINT:
                self.assertIn(position + 10, goals)
                self.assertEqual(goals[-1], position)
            else:
                self.assertIn(position + 1, goals)
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
                    elif joint != wave.GRIPPER:
                        self.assertTrue(all(goal == position for goal in goals))
                self.assert_clean(bus)

    def test_partial_enable_failure_releases_and_restores_all(self):
        bus = FakeBus(fail_enable="elbow_flex")
        with self.assertRaisesRegex(RuntimeError, "simulated"):
            self.run_wave(bus)
        self.assert_clean(bus)

    def test_gripper_moves_both_ways_with_wrist_and_returns_to_rest(self):
        bus = FakeBus()
        peak = 35 * wave.STEPS_PER_DEG
        self.run_wave(bus, samples=(peak, -peak, 0))
        center = bus.positions[wave.GRIPPER]
        goals = [event[2] for event in bus.events
                 if event[:2] == ("Goal_Position", wave.GRIPPER)]
        self.assertEqual(goals, [center, round(center + 3 * wave.STEPS_PER_DEG),
                                 round(center - 3 * wave.STEPS_PER_DEG), center, center])
        self.assert_clean(bus)

    def test_gripper_amplitude_is_adjustable(self):
        bus = FakeBus()
        self.run_wave(bus, extra_args=("--grip-amplitude", "2"),
                      samples=(35 * wave.STEPS_PER_DEG, 0))
        self.assertIn(("Goal_Position", wave.GRIPPER,
                       round(bus.positions[wave.GRIPPER] + 2 * wave.STEPS_PER_DEG)), bus.events)
        self.assert_clean(bus)

    def test_zero_grip_or_wrist_amplitude_holds_gripper_still(self):
        for option in ("--grip-amplitude", "--amplitude"):
            with self.subTest(option=option):
                bus = FakeBus()
                self.run_wave(bus, extra_args=(option, "0"), samples=(0, 0))
                goals = [event[2] for event in bus.events
                         if event[:2] == ("Goal_Position", wave.GRIPPER)]
                self.assertEqual(goals, [bus.positions[wave.GRIPPER]] * 2)
                self.assert_clean(bus)

    def test_gripper_motion_respects_narrow_or_degenerate_limits(self):
        for headroom in (5, 0):
            with self.subTest(headroom=headroom):
                bus = FakeBus()
                center = bus.positions[wave.GRIPPER]
                bus.limits[wave.GRIPPER] = [center - headroom, center + headroom]
                bus.original_limits[wave.GRIPPER] = list(bus.limits[wave.GRIPPER])
                peak = 35 * wave.STEPS_PER_DEG
                self.run_wave(bus, samples=(peak, -peak, 0))
                goals = [event[2] for event in bus.events
                         if event[:2] == ("Goal_Position", wave.GRIPPER)]
                self.assertEqual(min(goals), center - headroom)
                self.assertEqual(max(goals), center + headroom)
                self.assert_clean(bus)

    def test_invalid_grip_amplitude_is_rejected_before_connecting(self):
        for value in ("-1", "nan", "inf"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                bus = FakeBus()
                with self.assertRaises(SystemExit):
                    self.run_wave(bus, extra_args=("--grip-amplitude", value))
                self.assertFalse(bus.is_connected)
                self.assertEqual(bus.events, [])

    def test_interrupt_releases_and_restores_all(self):
        bus = FakeBus()
        self.run_wave(bus, interrupt=True)
        self.assert_clean(bus)

    def test_default_motion_is_continuous_with_requested_amplitude_and_period(self):
        bus = FakeBus()
        self.run_wave(bus, interrupt=True)
        self.assertEqual(self.motion_parameters, (math.inf, 35 * wave.STEPS_PER_DEG, 1.5))
        self.assert_clean(bus)

    def test_dry_run_never_writes_or_changes_torque(self):
        bus = FakeBus()
        self.run_wave(bus, dry=True)
        self.assertEqual(bus.events, [])
        self.assertFalse(bus.disconnected_with)

    def test_diagnostics_allow_healthy_motion(self):
        bus = FakeBus()
        self.run_wave(bus, diagnostics=True)
        self.assert_clean(bus)
        self.assertIn(("Goal_Position", wave.JOINT, bus.positions[wave.JOINT] + 10), bus.events)

    def test_elbow_fault_stops_without_sending_new_goals(self):
        bus = FakeBus()
        bus.telemetry["Status"] = 32
        self.run_wave(bus, diagnostics=True)
        self.assert_clean(bus)
        goals = [event for event in bus.events if event[0] == "Goal_Position"]
        self.assertEqual(len(goals), len(wave.MOTORS))  # only startup goals

    def test_fault_after_motion_stops_before_next_wrist_command(self):
        bus = FakeBus()
        write = bus.write

        def trip_after_motion(register, joint, value, **kwargs):
            write(register, joint, value, **kwargs)
            if register == "Goal_Position" and joint == wave.JOINT and value != bus.positions[joint]:
                bus.telemetry["Status"] = 32

        with patch.object(bus, "write", side_effect=trip_after_motion), \
             patch.object(wave.time, "monotonic", side_effect=[0, 0, 2]):
            self.run_wave(bus, diagnostics=True)
        self.assert_clean(bus)
        wrist_goals = [event[2] for event in bus.events
                       if event[:2] == ("Goal_Position", wave.JOINT)]
        self.assertEqual(wrist_goals, [bus.positions[wave.JOINT], bus.positions[wave.JOINT] + 10])


class ElbowDiagnosticTests(unittest.TestCase):
    def test_torque_loss_is_reported_without_writes(self):
        bus = FakeBus()
        bus.goals = dict(bus.positions)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertFalse(wave._check_elbow(bus, bus.positions["elbow_flex"]))
        self.assertIn("Torque_Enable=0", output.getvalue())
        self.assertEqual(bus.events, [])

    def test_failed_read_preserves_other_telemetry_and_error(self):
        bus = FakeBus()
        bus.enabled = set(wave.MOTORS)
        bus.goals = dict(bus.positions)
        read = bus.read

        def faulty_read(register, *args, **kwargs):
            if register == "Status":
                raise RuntimeError("servo overload response")
            return read(register, *args, **kwargs)

        with patch.object(bus, "read", side_effect=faulty_read), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertFalse(wave._check_elbow(bus, bus.positions["elbow_flex"]))
        self.assertIn("Present_Temperature=30", output.getvalue())
        self.assertIn("servo overload response", output.getvalue())
        self.assertEqual(bus.events, [])


class WaveTimingTests(unittest.TestCase):
    def test_continuous_motion_keeps_generating_until_interrupted(self):
        with patch.object(wave.time, "perf_counter", side_effect=[0, 0.375, 3600.375, KeyboardInterrupt]), \
             patch.object(wave.time, "sleep"):
            offsets = wave.wave_offsets(math.inf, 100, 1.5)
            self.assertAlmostEqual(next(offsets), 100)
            self.assertAlmostEqual(next(offsets), 100)
            with self.assertRaises(KeyboardInterrupt):
                next(offsets)

    def test_finite_motion_still_finishes_at_rest(self):
        with patch.object(wave.time, "perf_counter", side_effect=[0, 0.375, 1.5]), \
             patch.object(wave.time, "sleep"):
            offsets = list(wave.wave_offsets(1, 100, 1.5))
            self.assertAlmostEqual(offsets[0], 100)
            self.assertEqual(offsets[-1], 0)


if __name__ == "__main__":
    unittest.main()
