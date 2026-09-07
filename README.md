# LeRobot basic testing

Small scripts for a six-servo SO-arm using LeRobot's Feetech motor bus.
Run these in a Python environment with `lerobot` and its Feetech dependencies installed.

```sh
python hello_updown.py --dry-run
python hello_updown.py --waves 3 --amplitude 15 --period 1
```

`hello_updown.py` moves `wrist_flex` while the other five servos hold their
positions captured at startup. It returns the wrist to its starting position
and releases all servos on exit, including after Ctrl+C. Support the arm when
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
