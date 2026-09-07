# LeRobot basic testing

Small scripts for a six-servo SO-arm using LeRobot's Feetech motor bus.
Run these in a Python environment with `lerobot` and its Feetech dependencies installed.

```sh
python hello_updown.py
python hello_updown.py --dry-run --waves 3
python hello_updown.py --waves 5 --amplitude 20 --period 2
```

With no options, the arm waves continuously at ±35° with a 1.5-second period.
Press Ctrl+C to stop. Use `--waves N` for a finite number of waves, or
`--waves inf` to explicitly select continuous motion. The swing is trimmed
when necessary to stay within the encoder range.

`hello_updown.py` moves `wrist_flex` while the other five servos hold their
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
