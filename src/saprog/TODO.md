# TODO

## Move timeline execution into ROS

- [x] Define a ROS 2 action interface for executing a timeline, including steps, transition duration modes, waits, loop behavior, and the after-timeline torque choice.
- [x] Implement a ROS executor that validates goals, sends synchronized commands through the shared Dynamixel bus, monitors joint feedback, reports progress, and supports cancellation.
- [x] Update the Tkinter timeline editor to submit action goals, display feedback, and cancel execution; remove its playback and timing state machine.
- [x] Document ROS-owned scheduling, deadline failures, cancellation, launch behavior, and the non-hard-real-time limitation.
- [x] Build the ROS package, run the focused Python and bus regression tests, and verify the installed action interface imports.
- [ ] Add ROS action-server tests for scheduling, waits, loops, cancellation, goal rejection, and end-torque behavior.
- [ ] Validate timeline playback and overrun handling with the Dynamixel hardware.

## Later: ROS trajectory control

- [ ] Evaluate migrating the Dynamixel hardware access to `ros2_control` and using a trajectory controller for walking and continuous joint trajectories.
