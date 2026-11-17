# TODO

## Move timeline execution into ROS

- [ ] Define a ROS 2 action interface for executing a timeline, including steps, transition duration modes, waits, loop behavior, and the after-timeline torque choice.
- [ ] Implement a ROS executor that validates goals, sends commands through the existing position controller, monitors joint feedback, reports progress, and supports cancellation.
- [ ] Add tests for arrival-based and timed transitions, waits, loop behavior, torque handling, failures, and cancellation.
- [ ] Update the Tkinter timeline editor to submit action goals, display feedback, and cancel execution; remove its playback and timing state machine.
- [ ] Document how to launch the executor and editor together, and verify that the existing snapshot workflow still works.

## Later: ROS trajectory control

- [ ] Evaluate migrating the Dynamixel hardware access to `ros2_control` and using a trajectory controller for walking and continuous joint trajectories.
