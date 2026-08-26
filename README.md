# PincherX 100 Teach Pendant v4.1

This is a Cartesian jog correction for v4.

## Why v4 could feel wrong

v4 calculated every jog from the latest measured joint state. During/just after a
move, that state can be an intermediate pose. It also accepted the first valid IK
solution, which can sometimes switch kinematic branch.

That means +5 mm followed by -5 mm was not guaranteed to target exactly the same
Cartesian point.

## v4.1 changes

- Maintains a separate Cartesian target pose.
- `X +5 mm` modifies that target by +5 mm.
- A completed `X -5 mm` modifies the same target by -5 mm.
- Therefore equal opposite jogs return to the previous target.
- Rejects a new Cartesian jog while the previous arm move is still active.
- Evaluates all valid IK candidates and chooses the one requiring the smallest
  joint movement from the current configuration.
- Shows both:
  - Actual X/Y/Z/Pitch
  - Cartesian Target X/Y/Z/Pitch
- Adds `RESET TARGET = ACTUAL`.

## Test

Start from HOME and press `RESET TARGET = ACTUAL`.

Use:
- XYZ step = 5 mm
- Pitch step = 2 deg
- Move time = 1.5 s

Then test:

1. Note Target X.
2. Press X+ once. Target X must increase exactly 5.0 mm.
3. Wait until READY.
4. Press X- once. Target X must return exactly to the starting value.
5. Actual X should follow the target within normal servo/kinematic accuracy.

Repeat for Z, then Y.

PX100 is a 4-DOF arm. Y movement changes the required base yaw, so the waist can
rotate considerably even for a small Y jog. This is expected.

A Cartesian jog is still solved to a joint-space target; it is not a guaranteed
straight-line path.
