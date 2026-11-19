"""Run after sourcing the simulation install: verify spawn, gravity, and ground contact."""

import json
import math
import os
import signal
import subprocess
import tempfile
import time
import uuid
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def run(*args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=15)


def service(name, request_type, response_type, request):
    return run("gz", "service", "-s", "/world/saprog_simulation/" + name,
               "--reqtype", "gz.msgs." + request_type, "--reptype", "gz.msgs." + response_type,
               "--timeout", "5000", "--req", request)


def pose():
    output = run("gz", "topic", "-e", "-n", "1", "--json-output",
                 "-t", "/world/saprog_simulation/pose/info")
    # The CLI can deliver several queued callbacks before honoring --num.
    message = json.loads(output.splitlines()[-1])
    return next(p for p in message["pose"] if p["name"] == "saprog")["position"]


def step(count, target):
    response = service("control", "WorldControl", "Boolean", f"pause: true, multi_step: {count}")
    assert "data: true" in response, response
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        output = run("gz", "topic", "-e", "-n", "1", "--json-output",
                     "-t", "/world/saprog_simulation/stats")
        stats = json.loads(output.splitlines()[-1])
        if int(stats.get("iterations", 0)) >= target:
            return
    raise AssertionError(f"Physics did not reach {target} steps")


def main():
    os.environ["GZ_PARTITION"] = "saprog-check-" + uuid.uuid4().hex
    with tempfile.TemporaryDirectory(prefix="saprog-physics-check-") as directory:
        marker = Path(directory) / "ready"
        with (Path(directory) / "launch.log").open("w+") as log:
            process = subprocess.Popen([
                "ros2", "launch", "saprog_simulation", "simulation.launch.py", "gui:=false",
                f"ready_file:={marker}",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 45
                while not marker.exists() and time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError("Gazebo exited before spawn")
                    time.sleep(0.1)
                assert marker.exists(), "robot spawn timed out"
                # Scene service includes the complete imported model, even while paused.
                scene = service("scene/info", "Empty", "Scene", "")
                assert 'name: "saprog"' in scene, "spawn was accepted but model is absent"
                description = Path(get_package_share_directory("saprog_description"))
                robot = ET.parse(description / "urdf" / "URDF.urdf").getroot()
                for mesh in robot.findall("./link/visual/geometry/mesh"):
                    assert Path(mesh.attrib["filename"]).name in scene, mesh.attrib
                before = pose()
                assert abs(before.get("z", 0) - 0.5) < 0.01, before
                step(100, 100)
                falling = pose()
                assert 0.005 < before["z"] - falling["z"] < 0.15, (before, falling)
                step(3000, 3100)
                contact = pose()
                assert all(math.isfinite(v) for v in contact.values()), contact
                assert -0.05 < contact.get("z", 0) < 0.4, contact
                step(1000, 4100)
                settled = pose()
                assert abs(contact.get("z", 0) - settled.get("z", 0)) < 0.03, (contact, settled)
                print(f"PASS: paused spawn={before}, gravity={falling}, ground contact={settled}")
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-12000:])
                raise
            finally:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == "__main__":
    main()
