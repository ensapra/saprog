# Walk cycle

This folder contains the eight phase snapshots for an experimental, slow
four-beat crawl for the 12-servo quadruped. The looping timeline is
`sequences/walk_cycle.json`; its snapshot references are relative to the
project `snapshots/` directory.

## Observed limits and forward reach

The supplied Creator screenshot shows these limits:

| Servo group | IDs | Limit (rad) |
| --- | --- | --- |
| First joint | 11, 21, 31, 41 | -0.450 to 1.085 |
| Second (fore-aft) joint | 12, 22, 32, 42 | -0.962 to 1.085 |
| Third (knee) joint | 13, 23, 33, 43 | 0.266 to 0.880 |

You confirmed that decreasing the second joint angle moves the foot forward.
The gait now sweeps that joint from -0.90 rad at its forward extreme to
0.35 rad during stance. The forward target stays 0.062 rad inside the observed
lower limit instead of commanding the servo against its hard stop. First-joint
targets remain at 0.0 rad; knee targets remain at 0.40 and 0.58 rad.

## Leg map and motion

The first servo in each group is held at zero. The second and third servos are
the leg joints varied by this sequence:

- Front right: `servo_11`–`servo_13`
- Back right: `servo_21`–`servo_23`
- Front left: `servo_31`–`servo_33`
- Back left: `servo_41`–`servo_43`

The sequence offsets the same small hip sweep and knee lift between legs. The
intended footfall order is front right, back left, front left, back right. Each
leg spends most of its cycle in stance and briefly flexes its knee while
returning forward. Transitions wait for target arrival rather than sending the
next pose on a fixed timer.

## Before running

The sequence is a conservative starting point, not a kinematically verified
or hardware-tested gait. Joint direction and physical link geometry cannot be
determined from the snapshot files alone, and hardware limits are read from
the connected servos when the app opens.

Start from the existing `snapshots/stand.json` pose. Test with the robot
supported so its feet cannot catch or push against the floor, keep clear of
moving legs, and stop playback if a joint binds, the body tips, or a target is
outside the robot's observed range. Confirm the motion and adjust the targets
for the actual build before allowing it to walk under load.

Open `sequences/walk_cycle.json` in Timeline mode. It repeats until **Stop
sequence** is pressed and leaves torque on when playback is stopped or fails.
