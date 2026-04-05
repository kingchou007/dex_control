"""Kinesthetic Teaching for Franka Robot

Enables gravity compensation mode allowing manual guidance of the robot arm.
Publishes robot state via ZMQ for data collection.
"""

import time
import numpy as np
from termcolor import cprint

from dex_control.robot.robot_client import FrankaRobotClient
from dex_control.utils.zmq_publisher import ZMQStatePublisher


class KinestheticTeaching:
    """Kinesthetic teaching controller for Franka robot."""

    def __init__(self, ip: str, port: int, duration: float = 300.0,
                 impedance: float = 5.0, zmq_port: int = 5557,
                 publish_rate: float = 100.0):
        self.ip = ip
        self.port = port
        self.duration = duration
        self.impedance = impedance
        self.zmq_port = zmq_port
        self.publish_rate = publish_rate
        self.server_addr = f"tcp://{ip}:{port}"

    def run(self):
        """Run kinesthetic teaching session."""
        cprint("=== Kinesthetic Teaching Mode ===\n", "cyan", attrs=["bold"])

        # Initialize ZMQ publisher
        zmq_pub = ZMQStatePublisher(port=self.zmq_port)
        cprint(f"ZMQ state publisher on port {self.zmq_port}", "green")

        try:
            cprint(f"Connecting to robot at {self.server_addr}...", "yellow")
            robot = FrankaRobotClient(server_addr=self.server_addr)
            cprint("Connected!\n", "green")
        except Exception as e:
            cprint(f"Error connecting to robot: {e}", "red")
            zmq_pub.close()
            return

        cprint(f"Starting kinesthetic teaching for {self.duration} seconds...", "yellow")
        cprint(f"Joint impedance: {self.impedance} (0=soft, 50=stiff)", "yellow")
        cprint(f"Publishing state via ZMQ at {self.publish_rate} Hz", "green")
        cprint("You can now manually guide the robot arm.\n", "green", attrs=["bold"])

        try:
            robot.start_gravity_compensation(duration=self.duration, joint_impedance=self.impedance)

            cprint("Press Ctrl+C to stop and exit.\n", "magenta")

            # Monitor and publish robot state
            start_time = time.time()
            publish_period = 1.0 / self.publish_rate
            last_publish_time = 0

            while robot.is_gravity_compensation_active() and (time.time() - start_time) < self.duration:
                current_time = time.time()

                # Get robot state
                joints = robot.get_joint_positions()
                ee_pose = robot.get_ee_pose()

                # Publish via ZMQ at specified rate
                if current_time - last_publish_time >= publish_period:
                    try:
                        gripper_width = robot.get_gripper_width()
                    except Exception:
                        gripper_width = None
                    zmq_pub.publish(ee_pose, joints, gripper_width)
                    last_publish_time = current_time

                # Print current state
                joint_str = ", ".join([f"{j:.3f}" for j in joints])
                pos_str = f"[{ee_pose['t'][0]:.3f}, {ee_pose['t'][1]:.3f}, {ee_pose['t'][2]:.3f}]"

                elapsed = current_time - start_time
                remaining = self.duration - elapsed

                print(f"\r[{remaining:.0f}s] Joints: [{joint_str}] | EE: {pos_str}", end="", flush=True)

                time.sleep(0.01)

        except KeyboardInterrupt:
            cprint("\n\nStopping kinesthetic teaching...", "yellow")
        finally:
            robot.stop_gravity_compensation()
            cprint("Kinesthetic teaching ended.", "green")
            robot.close()
            zmq_pub.close()
