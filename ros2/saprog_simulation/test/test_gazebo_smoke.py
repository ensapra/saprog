"""Opt-in live Harmonic tests: SAPROG_RUN_GAZEBO_TESTS=1, source ROS first."""

import json
import os
import shutil
import sys
import time
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from saprog_simulation.session import GazeboSession  # noqa: E402


def synthetic_bounds():
    """Explicit synthetic input for the legacy direct-bounds command contract."""
    return {f"servo_{leg}{joint}": [-.25, 1.] for leg in (1, 2, 3, 4) for joint in (1, 2, 3)}


@unittest.skipUnless(
    os.environ.get("SAPROG_RUN_GAZEBO_TESTS") == "1" and shutil.which("gz") and sys.platform == "linux",
    "Enable live Gazebo tests in a sourced ROS Jazzy Linux environment",
)
class GazeboSmokeTests(unittest.TestCase):
    def test_import_gravity_pause_reset_and_cleanup(self):
        session = GazeboSession()
        self.addCleanup(session.stop)
        urdf = PACKAGE.parents[1] / "urdf/urdf/URDF.urdf"
        started = time.monotonic()
        session.start({"urdf_path": str(urdf), "paused": True, "spawn_height": 0.6,
                       "bounds_profile": json.loads((
                           PACKAGE / "profiles/offline-baseline.json"
                       ).read_text(encoding="utf-8"))})
        self.assertTrue(session.status()["running"])
        self.assertEqual(len(session.joints), 12)
        session.pause(False)
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            state = session.status()
            if state.get("sim_time", 0) >= 1.5 and "height" in state:
                break
            if not state["running"]:
                self.fail(f"Gazebo exited: {session.log_tail()}")
            time.sleep(0.1)
        else:
            self.fail(f"No gravity/pose feedback: {session.status()}\n{session.log_tail()}")
        self.assertLess(state["height"], 0.55, "The body should fall from its 0.60 m spawn height")
        print(f"\nGazebo import + gravity: {time.monotonic() - started:.2f}s wall; state={state}")
        session.pause(True)
        # Stats are deliberately throttled; allow the pause acknowledgement through.
        time.sleep(0.8)
        self.assertTrue(session.status()["paused"])
        frozen = session.status()["sim_time"]
        time.sleep(0.8)
        self.assertAlmostEqual(session.status()["sim_time"], frozen, places=2)
        old_process = session.server
        session.reset()
        self.assertIsNotNone(old_process.poll())
        self.assertTrue(session.status()["running"])
        self.assertTrue(session.status()["paused"])
        process = session.server
        session.stop()
        self.assertIsNotNone(process.poll())
        self.assertFalse(session.status()["running"])

    def test_ros_node_command_results_and_shutdown(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from saprog_simulation.node import PREFIX, SimulationNode
        from std_msgs.msg import String

        rclpy.init()
        node = SimulationNode()
        client = rclpy.create_node("saprog_simulation_smoke_client")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(client)
        replies = {}
        states = []
        publisher = client.create_publisher(String, f"{PREFIX}/command", 10)
        client.create_subscription(
            String, f"{PREFIX}/result", lambda message: replies.update(
                {json.loads(message.data)["id"]: json.loads(message.data)},
            ), 10,
        )
        client.create_subscription(String, f"{PREFIX}/status", lambda message: states.append(
            json.loads(message.data),
        ), 10)

        def wait_for(condition, seconds=35):
            deadline = time.monotonic() + seconds
            while not condition() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertTrue(condition(), f"ROS simulation request timed out: {replies}")

        def request(action, **options):
            request_id = str(len(replies))
            publisher.publish(String(data=json.dumps({"id": request_id, "action": action, **options})))
            wait_for(lambda: request_id in replies)
            return replies[request_id]

        try:
            wait_for(lambda: publisher.get_subscription_count() > 0)
            self.assertFalse(node.session.status()["running"])
            urdf = PACKAGE.parents[1] / "urdf/urdf/URDF.urdf"
            self.assertTrue(request("start", urdf_path=str(urdf), paused=True,
                                    servo_bounds=synthetic_bounds())["success"])
            wait_for(lambda: any(state.get("running") for state in states))
            self.assertFalse(request("unknown")["success"])
            self.assertTrue(request("resume")["success"])
            self.assertTrue(request("pause")["success"])
            process = node.session.server
            self.assertTrue(request("stop")["success"])
            self.assertIsNotNone(process.poll())
        finally:
            node.close()
            executor.shutdown()
            client.destroy_node()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
