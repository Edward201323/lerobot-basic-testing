#!/usr/bin/env python3
"""Wave hello by rolling the SO-arm's wrist back and forth.

Drives ONLY the wrist_roll joint (id 5) on a Feetech STS3215 bus. Every other
joint is left torque-off and limp, so nothing else on the arm can move.

Positions are written raw (uncalibrated), centered on wherever the wrist
already is, so this does not depend on the arm's calibration files.

    python hello.py                  # 3 waves, +/-30 deg
    python hello.py --dry-run        # print the motion, command nothing
"""

import argparse
import glob
import math
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

JOINT = "wrist_roll"
STEPS_PER_DEG = 4096 / 360  # STS3215: 4096 encoder steps per full turn
POS_MIN, POS_MAX = 0, 4095
FRAME_RATE = 50.0  # position commands per second

# The whole arm, so the bus can address it -- but only JOINT is ever energized.
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
    """Yield step offsets from center tracing `waves` back-and-forth rolls.

    The offset follows a sine so the wrist eases through each turnaround
    instead of slamming into it.
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
    """Draw the wrist as a tilt gauge leaning left or right."""
    middle = width // 2
    track = ["-"] * width
    track[middle] = "+"
    track[middle + round(degrees / amplitude_deg * middle)] = "#"
    hand = "\\" if degrees < -2 else "/" if degrees > 2 else "|"
    return f"  {hand}  [{''.join(track)}] {degrees:+6.1f} deg"


def main():
    parser = argparse.ArgumentParser(description="Wave hello with the arm's wrist.")
    parser.add_argument("--port", default=None, help="serial port (default: autodetect)")
    parser.add_argument("--waves", type=float, default=3, help="back-and-forth rolls (default: 3)")
    parser.add_argument("--amplitude", type=float, default=30.0, help="degrees to each side (default: 30)")
    parser.add_argument("--period", type=float, default=0.9, help="seconds per full roll (default: 0.9)")
    parser.add_argument("--dry-run", action="store_true", help="show the motion without commanding the arm")
    args = parser.parse_args()

    port = args.port or find_port()
    bus = FeetechMotorsBus(port, MOTORS)
    bus.connect(handshake=False)

    try:
        center = bus.read("Present_Position", JOINT, normalize=False)

        # Keep the whole swing inside the encoder's range, whatever the wrist's
        # resting angle happens to be.
        amplitude = min(
            args.amplitude,
            (center - POS_MIN) / STEPS_PER_DEG,
            (POS_MAX - center) / STEPS_PER_DEG,
        )
        if amplitude < args.amplitude:
            print(f"note: wrist rests near a limit, trimming swing to +/-{amplitude:.0f} deg")
        amplitude_steps = amplitude * STEPS_PER_DEG

        print(f"port {port} | {JOINT} at {center} | waving +/-{amplitude:.0f} deg")
        print("hello!")

        if not args.dry_run:
            # Park the goal on the current position BEFORE energizing, so the
            # servo holds still instead of snapping to a stale goal.
            bus.write("Goal_Position", JOINT, center, normalize=False)
            bus.enable_torque(JOINT)

        for offset in wave_offsets(args.waves, amplitude_steps, args.period):
            if not args.dry_run:
                target = round(center + offset)
                bus.write("Goal_Position", JOINT, target, normalize=False)
            print(render(offset / STEPS_PER_DEG, amplitude), end="\r", flush=True)
        print()

    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        if not args.dry_run and bus.is_connected:
            try:
                bus.write("Goal_Position", JOINT, center, normalize=False)
                time.sleep(0.4)  # let it coast home before going limp
            except Exception:
                pass
        bus.disconnect(disable_torque=True)  # leave the arm limp, as found


if __name__ == "__main__":
    main()
