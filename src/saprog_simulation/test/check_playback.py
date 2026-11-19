"""Exercise desktop playback commands against a real headless Gazebo world."""

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/saprog/scripts"))
from saprog_desktop.simulation_playback import SimulationRobot  # noqa: E402
from saprog_desktop.simulation_session import SimulationSession  # noqa: E402


def wait(session, predicate, seconds=30):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        session.poll()
        if predicate():
            return
        if session.state == "error":
            raise AssertionError((session.message, session.playback))
        time.sleep(0.05)
    raise AssertionError((session.state, session.playback))


def main():
    session = SimulationSession(ROOT)
    with tempfile.TemporaryDirectory(prefix="saprog-playback-check-") as directory:
        try:
            assert session.start(gui=False)
            wait(session, lambda: session.state == "running")
            print("PASS: runtime actuators and simulated feedback", flush=True)
            env = os.environ.copy()
            env["GZ_PARTITION"] = session.partition

            def control(paused):
                subprocess.run([
                    "gz", "service", "-s", "/world/saprog_simulation/control",
                    "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "5000", "--req", f"pause: {str(paused).lower()}",
                ], env=env, check=True, timeout=10, stdout=subprocess.DEVNULL)

            snapshot = Path(directory) / "pose.json"
            invalid = Path(directory) / "invalid.json"
            invalid.write_text("{")
            request = session.command("play", path=invalid, kind="snapshot", snapshots=directory)
            wait(session, lambda: session.playback.get("request") == request)
            assert session.playback["state"] == "idle", session.playback
            assert "Cannot play" in session.playback["message"], session.playback
            print("PASS: invalid documents are rejected without starting playback", flush=True)
            robot = SimulationRobot(ROOT / "urdf/urdf/URDF.urdf")
            positions = {name: max(low, min(high, 0.1)) if low < 0 else (low + high) / 2
                         for name, (low, high) in robot.bounds.items()}
            snapshot.write_text(json.dumps({"schema_version": 1, "name": "test", "positions": positions}))
            request = session.command("play", path=snapshot, kind="snapshot", snapshots=directory)
            wait(session, lambda: session.playback.get("request") == request)
            assert session.playback["state"] == "playing", session.playback
            paused_at = session.playback["sim_time"]
            time.sleep(0.5)
            session.poll()
            assert session.playback["sim_time"] == paused_at
            print("PASS: snapshot queues while paused; simulation time is frozen", flush=True)
            control(False)
            wait(session, lambda: session.playback.get("state") in ("complete", "error"), 45)
            assert session.playback["state"] == "complete", session.playback
            assert abs(session.playback["positions"]["front_right_joint_13"] - positions["servo_13"]) < 0.05
            print("PASS: snapshot reaches its target using simulated feedback", flush=True)
            control(True)
            sequence = Path(directory) / "sequence.json"
            sequence.write_text(json.dumps({"kind": "saprog_timeline", "schema_version": 2,
                "on_end": "off", "loop": False, "steps": [
                    {"snapshot": "pose.json", "motion": {"speed": 300, "torque": 700},
                     "duration": {"mode": "seconds", "seconds": 0.3}},
                    {"kind": "wait", "seconds": 0.3},
                ]}))
            request = session.command("play", path=sequence, kind="sequence", snapshots=directory)
            wait(session, lambda: session.playback.get("request") == request)
            control(False)
            wait(session, lambda: session.playback.get("state") == "complete")
            assert "released" in session.playback["message"], session.playback
            print("PASS: sequence references, timed motion, waits and end release", flush=True)
            control(True)
            request = session.command("play", path=snapshot, kind="snapshot", snapshots=directory)
            wait(session, lambda: session.playback.get("request") == request)
            request = session.command("stop")
            wait(session, lambda: session.playback.get("request") == request)
            assert session.playback["state"] == "idle"
            print("PASS: Stop is acknowledged while Gazebo is paused", flush=True)
        finally:
            session.close()


if __name__ == "__main__":
    main()
