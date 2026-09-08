#!/usr/bin/env python3
"""Wave hello by flexing the SO-arm's wrist up and down.

The pose measured at startup becomes this run's rest pose. Moves wrist_flex
(id 4) relative to its rest angle and gently opens/closes the gripper in sync.
The remaining joints hold their rest positions. Returns to the captured rest
pose, then releases all joints on exit.

wrist_flex ships with a degenerate firmware limit on this arm
(Min_Position_Limit == Max_Position_Limit == 2046, i.e. zero allowed travel),
so this script temporarily widens that limit to fit the swing and restores the
original values on the way out. The widening happens while the joint is still
limp -- energizing first would snap the wrist to the nearest limit.

    python jerkoff.py                # continuous waves, +/-35 deg, 1.5s period
    python jerkoff.py --waves 3      # stop after 3 waves
    python jerkoff.py --dry-run      # print the motion, command nothing
    python jerkoff.py --diagnostics  # report elbow telemetry once per second
"""

import argparse
import glob
import math
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

JOINT = "wrist_flex"
GRIPPER = "gripper"
STEPS_PER_DEG = 4096 / 360  # STS3215: 4096 encoder steps per full turn
POS_MIN, POS_MAX = 0, 4095
FRAME_RATE = 50.0  # position commands per second
MARGIN = 40  # extra steps of headroom around the swing, ~3.5 deg
WRITE_RETRIES = 3  # Feetech buses occasionally return a corrupted status packet
MAX_CONSECUTIVE_DROPS = 10  # ~0.2s of silence means the bus is really gone
DIAGNOSTIC_INTERVAL = 1.0  # seconds between elbow telemetry snapshots

MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}


def find_port():
    """Return the arm's serial port, preferring the USB-serial bridge."""
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    if not ports:
        sys.exit("No arm found: no /dev/cu.usbmodem* port. Is the arm plugged in and powered?")
    return ports[0]


def wave_offsets(waves, amplitude_steps, period):
    """Yield step offsets from center tracing `waves` up-and-down flexes.

    The offset follows a sine so the wrist eases through each turnaround
    instead of slamming into it. `waves=math.inf` runs until interrupted.
    """
    duration = waves * period
    start = time.perf_counter()
    elapsed = 0.0
    while elapsed < duration:
        elapsed = time.perf_counter() - start
        yield amplitude_steps * math.sin(2 * math.pi * elapsed / period)
        time.sleep(1 / FRAME_RATE)
    yield 0.0  # settle back to center


def render(degrees, amplitude_deg, width=31):
    """Draw the flex as a gauge, down on the left and up on the right."""
    middle = width // 2
    track = ["-"] * width
    track[middle] = "+"
    track[middle + (round(degrees / amplitude_deg * middle) if amplitude_deg else 0)] = "#"
    hand = "v" if degrees < -2 else "^" if degrees > 2 else "-"
    return f"  {hand}  down [{''.join(track)}] up {degrees:+6.1f} deg"



def _park(bus, rest_positions):
    """Best effort: return every joint to this run's captured rest pose."""
    for joint, position in rest_positions.items():
        _safely(bus.write, "Goal_Position", joint, position,
                normalize=False, num_retry=WRITE_RETRIES)
    time.sleep(0.4)  # let it coast home before going limp


def _release(bus):
    """Best effort: drop torque, which also unlocks EEPROM (Lock=0)."""
    for joint in MOTORS:
        _safely(bus.disable_torque, joint, num_retry=WRITE_RETRIES)


def _restore_limits(bus, joint, saved):
    """Put the firmware position limits back, verifying by read-back.

    This is the step that must not be skipped: leaving widened limits behind
    silently changes the arm's configuration for every later session.
    """
    lo, hi = saved
    for attempt in range(4):
        try:
            bus.disable_torque(joint, num_retry=WRITE_RETRIES)  # ensure Lock=0
            bus.write("Min_Position_Limit", joint, lo, normalize=False, num_retry=WRITE_RETRIES)
            bus.write("Max_Position_Limit", joint, hi, normalize=False, num_retry=WRITE_RETRIES)
            back = (
                bus.read("Min_Position_Limit", joint, normalize=False),
                bus.read("Max_Position_Limit", joint, normalize=False),
            )
            if back == (lo, hi):
                print(f"restored {joint} limits to {lo}-{hi}")
                return True
        except Exception:
            pass
        time.sleep(0.2 * (attempt + 1))
    print(
        f"\nWARNING: could not restore {joint} limits to {lo}-{hi}. "
        f"The arm may still have widened limits. Reset these limits before relying on the arm."
    )
    return False


def _safely(step, *args, **kwargs):
    """Run one cleanup step; never let its failure skip the steps after it."""
    try:
        step(*args, **kwargs)
    except Exception as e:
        print(f"\nwarning: {step.__name__} failed ({type(e).__name__}: {e})")


def _check_elbow(bus, rest_position):
    """Print raw telemetry; return False on a fault or unreadable feedback.

    Read each field independently so one failed read does not hide the rest.
    Goal writes can clear protection on these servos, so diagnostics only read.
    """
    values = {}
    errors = []
    for register in ("Torque_Enable", "Status", "Present_Position", "Goal_Position",
                     "Present_Load", "Present_Temperature", "Present_Voltage", "Present_Current"):
        try:
            values[register] = bus.read(register, "elbow_flex", normalize=False,
                                        num_retry=WRITE_RETRIES)
        except Exception as error:
            errors.append(f"{register}: {type(error).__name__}: {error}")
    fields = [f"rest={rest_position}"]
    fields.extend(f"{name}={value}" for name, value in values.items())
    if "Present_Position" in values:
        fields.append(f"error_deg={(values['Present_Position'] - rest_position) / STEPS_PER_DEG:+.1f}")
    print("\nelbow telemetry (raw registers): " + " | ".join(fields), flush=True)
    for error in errors:
        print(f"elbow read failed: {error}", flush=True)
    healthy = not errors and values.get("Torque_Enable") == 1 and values.get("Status") == 0
    if not healthy:
        print("Elbow fault, torque loss, or unreadable feedback; stopping and releasing the arm. "
              "Skipping return-to-rest commands to avoid clearing servo protection.", flush=True)
    return healthy


def main():
    parser = argparse.ArgumentParser(description="Wave hello by flexing the wrist up and down.")
    parser.add_argument("--port", default=None, help="serial port (default: autodetect)")
    parser.add_argument("--waves", type=float, default=math.inf,
                        help="up-and-down flexes; inf runs until Ctrl+C (default: inf)")
    parser.add_argument("--amplitude", type=float, default=35.0, help="degrees each way (default: 35)")
    parser.add_argument("--grip-amplitude", type=float, default=3.0,
                        help="gripper servo degrees each way, within firmware limits; 0 holds still (default: 3)")
    parser.add_argument("--period", type=float, default=1.5, help="seconds per full flex (default: 1.5)")
    parser.add_argument("--dry-run", action="store_true", help="show the motion without commanding the arm")
    parser.add_argument("--diagnostics", action="store_true",
                        help="report elbow telemetry every second; stop on a fault, torque loss, or read failure")
    args = parser.parse_args()
    if math.isnan(args.waves) or not all(math.isfinite(value) for value in
                                       (args.amplitude, args.grip_amplitude, args.period)):
        parser.error("waves must be a number or inf; amplitudes and period must be finite")
    if args.waves < 0 or args.amplitude < 0 or args.grip_amplitude < 0 or args.period <= 0:
        parser.error("waves and amplitudes must be nonnegative; period must be positive")

    port = args.port or find_port()
    bus = FeetechMotorsBus(port, MOTORS)
    bus.connect(handshake=False)

    center = None
    saved_limits = {}
    wrist_enabled = False
    elbow_fault = False
    try:
        # Capture once per run, before sending any commands. No fixed home
        # angle or saved pose overrides the position the user starts from.
        rest_positions = {
            joint: bus.read("Present_Position", joint, normalize=False)
            for joint in MOTORS
        }
        if any(not POS_MIN <= position <= POS_MAX for position in rest_positions.values()):
            raise ValueError("A joint position is outside the single-turn encoder range")
        center = rest_positions[JOINT]

        # Keep the swing inside the encoder's range, wherever the wrist rests.
        amplitude = min(
            args.amplitude,
            (center - POS_MIN) / STEPS_PER_DEG,
            (POS_MAX - center) / STEPS_PER_DEG,
        )
        if amplitude < args.amplitude:
            print(f"note: wrist rests near a limit, trimming swing to +/-{amplitude:.0f} deg")
        amplitude_steps = amplitude * STEPS_PER_DEG
        grip_center = rest_positions[GRIPPER]
        grip_limits = (
            bus.read("Min_Position_Limit", GRIPPER, normalize=False),
            bus.read("Max_Position_Limit", GRIPPER, normalize=False),
        )
        # Use raw servo angles, not normalized opening percentages. Respect
        # the gripper's existing travel limits when adding motion.
        grip_steps = min(
            args.grip_amplitude * STEPS_PER_DEG,
            max(0, grip_center - max(POS_MIN, grip_limits[0])),
            max(0, min(POS_MAX, grip_limits[1]) - grip_center),
        ) if amplitude_steps else 0
        if grip_steps < args.grip_amplitude * STEPS_PER_DEG:
            print("note: trimming gripper motion to fit its limits and wrist motion")

        print(f"port {port} | {JOINT} at {center} | flexing +/-{amplitude:.0f} deg")
        print(f"gripper at {grip_center} | opening/closing +/-{grip_steps / STEPS_PER_DEG:.1f} servo deg")
        print("rest pose (encoder steps): " + ", ".join(
            f"{joint}={position}" for joint, position in rest_positions.items()
        ))
        print("hello!")
        if math.isinf(args.waves):
            print("waving continuously; press Ctrl+C to return to rest and release")

        if not args.dry_run:
            for joint, position in rest_positions.items():
                if joint == GRIPPER:
                    lo, hi = grip_limits
                else:
                    lo = bus.read("Min_Position_Limit", joint, normalize=False)
                    hi = bus.read("Max_Position_Limit", joint, normalize=False)
                swing = amplitude_steps if joint == JOINT else 0
                need_lo = max(POS_MIN, round(position - swing) - MARGIN)
                need_hi = min(POS_MAX, round(position + swing) + MARGIN)

                if lo > need_lo or hi < need_hi:
                    # Make the captured position reachable before energizing.
                    # Preserve the original range when extending either end.
                    saved_limits[joint] = (lo, hi)
                    new_lo, new_hi = min(lo, need_lo), max(hi, need_hi)
                    print(f"widening {joint} limits {lo}-{hi} -> {new_lo}-{new_hi} (restored on exit)")
                    bus.disable_torque(joint, num_retry=WRITE_RETRIES)
                    bus.write("Min_Position_Limit", joint, new_lo, normalize=False, num_retry=WRITE_RETRIES)
                    bus.write("Max_Position_Limit", joint, new_hi, normalize=False, num_retry=WRITE_RETRIES)

                bus.write("Goal_Position", joint, position, normalize=False, num_retry=WRITE_RETRIES)

            # Each stationary servo maintains its goal internally while the
            # wrist and gripper receive changing goals. Energize the wrist last.
            for joint in MOTORS:
                if joint != JOINT:
                    bus.enable_torque(joint, num_retry=WRITE_RETRIES)
            bus.enable_torque(JOINT, num_retry=WRITE_RETRIES)
            wrist_enabled = True
            print("shoulder, elbow, and wrist roll holding their starting positions")

        drops = 0
        next_diagnostic = 0.0
        for offset in wave_offsets(args.waves, amplitude_steps, args.period):
            grip_offset = offset / amplitude_steps * grip_steps if amplitude_steps else 0
            if not args.dry_run:
                if args.diagnostics and time.monotonic() >= next_diagnostic:
                    if not _check_elbow(bus, rest_positions["elbow_flex"]):
                        elbow_fault = True
                        break
                    next_diagnostic = time.monotonic() + DIAGNOSTIC_INTERVAL
                try:
                    bus.write("Goal_Position", JOINT, round(center + offset),
                              normalize=False, num_retry=WRITE_RETRIES)
                    if grip_steps:
                        bus.write("Goal_Position", GRIPPER, round(grip_center + grip_offset),
                                  normalize=False, num_retry=WRITE_RETRIES)
                    drops = 0
                except Exception:
                    # One glitched packet should cost a frame, not the wave.
                    drops += 1
                    if drops >= MAX_CONSECUTIVE_DROPS:
                        print("\nbus went quiet, stopping early")
                        break
            print(render(offset / STEPS_PER_DEG, amplitude)
                  + f" | grip {grip_offset / STEPS_PER_DEG:+.1f} servo deg", end="\r", flush=True)
        print()

    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        if not args.dry_run and bus.is_connected:
            # Each step is independent: a failure in one must not skip the rest.
            if wrist_enabled and not elbow_fault:
                _safely(_park, bus, rest_positions)
            _safely(_release, bus)
            for joint, limits in saved_limits.items():
                _safely(_restore_limits, bus, joint, limits)
        _safely(bus.disconnect, not args.dry_run)


if __name__ == "__main__":
    main()
