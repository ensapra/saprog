"""Exercise the real controller on a virtual AX bus; never opens robot hardware."""

import os
import pty
import select
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


def check_latency(executable, config):
    master, slave = pty.openpty()
    registers = {servo: bytearray(64) for servo in (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)}
    for data in registers.values():
        data[8:10] = (1023).to_bytes(2, "little")
        data[36:38] = (512).to_bytes(2, "little")
    stop = threading.Event()
    received = []

    def serve():
        buffer = bytearray()
        while not stop.is_set():
            if not select.select([master], [], [], 0.05)[0]:
                continue
            buffer.extend(os.read(master, 1024))
            while len(buffer) >= 6:
                if buffer[:2] != b"\xff\xff":
                    del buffer[0]
                    continue
                size = buffer[3] + 4
                if len(buffer) < size:
                    break
                packet = buffer[:size]
                del buffer[:size]
                servo, instruction = packet[2], packet[4]
                if servo not in registers:
                    continue
                data = registers[servo]
                if instruction == 2:
                    address, count = packet[5:7]
                    # A small USB response delay makes a full sweep measurably slower
                    # than a single callback, so executor monopolization is detectable.
                    time.sleep(0.002)
                    reply = bytearray([servo, count + 2, 0]) + data[address:address + count]
                    reply.append((~sum(reply)) & 255)
                    os.write(master, b"\xff\xff" + reply)
                elif instruction == 3:
                    address = packet[5]
                    data[address:address + len(packet[6:-1])] = packet[6:-1]
                    received.append((time.monotonic(), servo, address))

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    domain = 100 + os.getpid() % 100
    rclpy.init(domain_id=domain)
    node = rclpy.create_node("latency_regression")
    samples = []
    _subscription = node.create_subscription(JointState, "/saprog/joint_states",
                                            lambda msg: samples.append((time.monotonic(), msg)), 1)
    publisher = node.create_publisher(JointState, "/saprog/position_command", 10)
    client = node.create_client(SetBool, "/saprog/servo_11/torque")
    process = subprocess.Popen([
        executable, "--ros-args", "--params-file", config,
        "-p", "device_name:=" + os.ttyname(slave), "-p", "poll_rate:=20.0",
    ], env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=subprocess.DEVNULL,
       stderr=subprocess.PIPE)

    def wait_for(predicate, timeout=5):
        end = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None:
                raise AssertionError(process.stderr.read().decode())
            if time.monotonic() > end:
                raise AssertionError("Timed out waiting for controller")
            rclpy.spin_once(node, timeout_sec=0.002)

    try:
        wait_for(lambda: len(samples) >= 5 and client.service_is_ready()
                 and publisher.get_subscription_count() > 0)
        intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        # Allow scheduler jitter on a shared WSL desktop; the former 1 Hz sweep
        # still fails this bound by a wide margin.
        assert statistics.median(intervals) < 0.2, intervals
        assert all(len(message.name) == 12 for _, message in samples)
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name, message.position, message.effort = ["servo_11"], [0.2], [600.0]
        started = time.monotonic()
        publisher.publish(message)
        wait_for(lambda: any(t >= started and address == 24 for t, _, address in received))
        on_latency = time.monotonic() - started
        wait_for(lambda: any(t >= started and msg.effort[msg.name.index("servo_11")] == 1.0
                            for t, msg in samples))
        on_confirmation = time.monotonic() - started
        started = time.monotonic()
        future = client.call_async(SetBool.Request(data=False))
        wait_for(future.done)
        off_latency = time.monotonic() - started
        assert future.result().success, future.result().message
        wait_for(lambda: any(t >= started and msg.effort[msg.name.index("servo_11")] == 0.0
                            for t, msg in samples))
        off_confirmation = time.monotonic() - started
        assert max(on_latency, off_latency) < 0.2, (on_latency, off_latency)
        assert max(on_confirmation, off_confirmation) < 0.4, (on_confirmation, off_confirmation)
        print(f"Virtual bus: sweep {statistics.median(intervals)*1000:.1f} ms; "
              f"Fix write {on_latency*1000:.1f} ms, confirmed {on_confirmation*1000:.1f} ms; "
              f"Release response {off_latency*1000:.1f} ms, confirmed {off_confirmation*1000:.1f} ms")
    finally:
        process.terminate()
        process.wait(timeout=5)
        process.stderr.close()
        node.destroy_node()
        rclpy.shutdown()
        stop.set()
        worker.join(timeout=1)
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    check_latency(*sys.argv[1:])
