"""Opt-in live ROS smoke test. Uses a separate ROS domain and owns its node process."""

import json
import os
import signal
import subprocess
import time
import uuid

import rclpy
from std_msgs.msg import String


def main():
    os.environ["ROS_DOMAIN_ID"] = "83"
    rclpy.init()
    node = rclpy.create_node("single_leg_smoke_test")
    state, results = {}, {}
    max_error = 0.0

    def receive_state(message):
        nonlocal max_error
        state.clear()
        state.update(json.loads(message.data))
        max_error = max(max_error, state.get("closure_error", 0))

    def receive_result(message):
        result = json.loads(message.data)
        results[result["id"]] = result

    node.create_subscription(String, "/saprog/leg_test/status", receive_state, 10)
    node.create_subscription(String, "/saprog/leg_test/result", receive_result, 10)
    publisher = node.create_publisher(String, "/saprog/leg_test/command", 10)
    process = subprocess.Popen(["ros2", "run", "saprog_leg_test", "leg_test_node"],
                               start_new_session=True)

    def wait_for(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if process.poll() is not None:
                raise AssertionError(f"Single-leg node exited: {process.returncode}")
            if predicate():
                return
        raise AssertionError(f"Timed out; last state: {state}")

    def command(action, **options):
        request_id = uuid.uuid4().hex
        message = String()
        message.data = json.dumps(dict(options, id=request_id, action=action))
        publisher.publish(message)
        wait_for(lambda: request_id in results)
        result = results.pop(request_id)
        assert result["success"], result

    try:
        wait_for(lambda: publisher.get_subscription_count() > 0 and bool(state))
        command("start", config={"real_time_factor": 2, "foot_load": 1})
        wait_for(lambda: state.get("running") and state.get("sim_time", 0) > 0.1)
        command("target", angle=0.65)
        wait_for(lambda: abs(state.get("servo_angle", 0) - 0.65) < 0.025)
        command("sweep", lower=0.3, upper=0.8, period=2)
        wait_for(lambda: state.get("sweeping") is True)
        initial = state["sim_time"]
        wait_for(lambda: state.get("sim_time", 0) > initial + 2)
        assert not state.get("error"), state
        command("pause")
        wait_for(lambda: state.get("paused") is True)
        paused_time = state["sim_time"]
        until = time.monotonic() + 0.2
        wait_for(lambda: time.monotonic() >= until)
        assert state["sim_time"] == paused_time, state
        command("reset")
        wait_for(lambda: state.get("sim_time") == 0 and state.get("paused"))
        command("resume")
        wait_for(lambda: state.get("sim_time", 0) > 0.1)
        command("stop")
        wait_for(lambda: state.get("running") is False)
        assert max_error < 1e-5, max_error
        print(json.dumps({"result": "PASS", "max_closure_error_mm": max_error * 1000,
                          "actions": "start, target, loaded sweep, pause, reset, resume, stop"}))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
