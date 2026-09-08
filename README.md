# LeRobot basic testing

Small scripts for a six-servo SO-arm using LeRobot's Feetech motor bus.
Run these in a Python environment with `lerobot` and its Feetech dependencies installed.

```sh
python jerkoff.py
python jerkoff.py --dry-run --waves 3
python jerkoff.py --waves 5 --amplitude 20 --period 2
```

With no options, the arm waves continuously at ±35° with a 1.5-second period.
Press Ctrl+C to stop. Use `--waves N` for a finite number of waves, or
`--waves inf` to explicitly select continuous motion. The swing is trimmed
when necessary to stay within the encoder range.

`jerkoff.py` moves `wrist_flex` while the other five servos hold their
positions captured at startup. Whatever pose you start the program in becomes
the rest pose for that run: the wrist waves around its captured angle, and all
joints are commanded back to that pose before release, including after Ctrl+C.
Each new run captures a fresh rest pose; it does not change calibration or save
a permanent home position. The captured encoder values are printed at startup.
Support the arm when
it releases. Any firmware position limits temporarily widened for the motion
or holding positions are restored during cleanup.

`--dry-run` reads the connected arm's positions and prints the motion without
writing goals, changing limits, or changing torque. Use `--port` to select a
serial device; automatic detection looks for macOS USB serial ports.

`hello.py` is the original wrist-roll example; it only enables the wrist-roll
servo and does not provide holding torque for the other joints.

Run the simulated bus tests without hardware or LeRobot installed:

```sh
python3 -m unittest discover -s tests -v
```

To investigate the elbow losing its hold, enable telemetry:

```sh
python jerkoff.py --diagnostics
```

This prints the elbow's torque state, fault status, actual and goal positions,
load, temperature, voltage, and current once per second. Register values are
raw; `error_deg` is the measured deviation from the captured rest position.
Save the terminal output around the failure for diagnosis. This mode stops
and releases the arm if torque is off, status is nonzero, or any telemetry
read fails after retries. It skips return-to-rest goal writes on these stops
because a new position command can clear servo protection. Support the arm
before running. Diagnostics do not establish the cause or fix a hardware fault.

The register names come from [LeRobot's Feetech control table](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py).
Feetech documents overload, overcurrent, voltage, and thermal protection in its
[STS3215 description](https://www.feetechrc.com/20210430-56680.html).
