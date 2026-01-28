"""Kinesthetic Teaching Example for Franka Robot

Enables gravity compensation mode allowing manual guidance of the robot arm.
Publishes robot state to ROS2 topics while teaching for recording.

Usage:
    python examples/4_kinesthetic_teaching.py [OPTIONS]

Options:
    --ip: Robot server IP address (default: 192.168.1.6)
    --port: Robot server port (default: 4242)
    --duration: Duration in seconds (default: 300)
    --impedance: Joint impedance (default: 5.0, higher = stiffer)
    --ros2-rate: ROS2 publish rate in Hz (default: 100)

ROS2 Topics Published:
    /franka/ee_pose (geometry_msgs/PoseStamped): End-effector pose
    /franka/joint_states (sensor_msgs/JointState): Joint positions
    /franka/gripper_width (std_msgs/Float32): Gripper width
"""

import time
import click
from termcolor import cprint
import numpy as np

from dex_control.robot.robot_client import FrankaRobotClient

# Optional ROS2 support
try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    cprint("Warning: ROS2 not available. Install rclpy to enable publishing.", "yellow")


DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242


class ROS2Publisher:
    """Simple ROS2 publisher for kinesthetic teaching."""

    def __init__(self, rate: float = 30.0):
        if not ROS2_AVAILABLE:
            self.enabled = False
            return

        rclpy.init()
        self.node = rclpy.create_node('kinesthetic_teaching_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.ee_pose_pub = self.node.create_publisher(
            PoseStamped, '/franka/ee_pose', qos_profile
        )
        self.joint_state_pub = self.node.create_publisher(
            JointState, '/franka/joint_states', qos_profile
        )
        self.gripper_width_pub = self.node.create_publisher(
            Float32, '/franka/gripper_width', qos_profile
        )

        self.enabled = True
        cprint(f"ROS2 publisher initialized at {rate} Hz", "green")

    def publish(self, ee_pose: dict, joint_positions: np.ndarray, gripper_width: float = None):
        """Publish robot state to ROS2 topics."""
        if not self.enabled:
            return

        now = self.node.get_clock().now().to_msg()

        # Publish end-effector pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.pose.position.x = float(ee_pose['t'][0])
        pose_msg.pose.position.y = float(ee_pose['t'][1])
        pose_msg.pose.position.z = float(ee_pose['t'][2])
        pose_msg.pose.orientation.x = float(ee_pose['q'][0])
        pose_msg.pose.orientation.y = float(ee_pose['q'][1])
        pose_msg.pose.orientation.z = float(ee_pose['q'][2])
        pose_msg.pose.orientation.w = float(ee_pose['q'][3])
        self.ee_pose_pub.publish(pose_msg)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.header.frame_id = "panda_link0"
        joint_msg.name = [f'panda_joint{i+1}' for i in range(7)]
        joint_msg.position = [float(q) for q in joint_positions]
        self.joint_state_pub.publish(joint_msg)

        # Publish gripper width
        if gripper_width is not None:
            width_msg = Float32()
            width_msg.data = float(gripper_width)
            self.gripper_width_pub.publish(width_msg)

    def shutdown(self):
        """Shutdown ROS2."""
        if self.enabled:
            self.node.destroy_node()
            rclpy.shutdown()
            cprint("ROS2 publisher stopped", "yellow")


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--duration", default=300.0, help="Duration in seconds")
@click.option("--impedance", default=5.0, help="Joint impedance (0=soft, 5-10=medium, 50=stiff)")
@click.option("--ros2-rate", default=100.0, help="ROS2 publish rate in Hz")
def main(ip, port, duration, impedance, ros2_rate):
    """Kinesthetic Teaching - Enable gravity compensation for manual guidance."""

    server_addr = f"tcp://{ip}:{port}"

    cprint("=== Kinesthetic Teaching Mode ===\n", "cyan", attrs=["bold"])

    # Initialize ROS2 publisher
    ros2_pub = ROS2Publisher(rate=ros2_rate)

    try:
        cprint(f"Connecting to robot at {server_addr}...", "yellow")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Connected!\n", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        ros2_pub.shutdown()
        return

    cprint(f"Starting kinesthetic teaching for {duration} seconds...", "yellow")
    cprint(f"Joint impedance: {impedance} (0=soft, 50=stiff)", "yellow")
    if ros2_pub.enabled:
        cprint(f"Publishing to ROS2 at {ros2_rate} Hz", "green")
    cprint("You can now manually guide the robot arm.\n", "green", attrs=["bold"])

    try:
        robot.start_gravity_compensation(duration=duration, joint_impedance=impedance)

        cprint("Press Ctrl+C to stop and exit.\n", "magenta")

        # Monitor and publish robot state
        start_time = time.time()
        publish_period = 1.0 / ros2_rate
        last_publish_time = 0

        while robot.is_gravity_compensation_active() and (time.time() - start_time) < duration:
            current_time = time.time()

            # Get robot state
            joints = robot.get_joint_positions()
            ee_pose = robot.get_ee_pose()

            # Publish to ROS2 at specified rate
            if current_time - last_publish_time >= publish_period:
                try:
                    gripper_width = robot.get_gripper_width()
                except Exception:
                    gripper_width = None
                ros2_pub.publish(ee_pose, joints, gripper_width)
                last_publish_time = current_time

            # Print current state (at lower rate for display)
            joint_str = ", ".join([f"{j:.3f}" for j in joints])
            pos_str = f"[{ee_pose['t'][0]:.3f}, {ee_pose['t'][1]:.3f}, {ee_pose['t'][2]:.3f}]"

            elapsed = current_time - start_time
            remaining = duration - elapsed

            print(f"\r[{remaining:.0f}s] Joints: [{joint_str}] | EE: {pos_str}", end="", flush=True)

            time.sleep(0.01)  # Small sleep to avoid busy loop

    except KeyboardInterrupt:
        cprint("\n\nStopping kinesthetic teaching...", "yellow")
    finally:
        robot.stop_gravity_compensation()
        cprint("Kinesthetic teaching ended.", "green")
        robot.close()
        ros2_pub.shutdown()


if __name__ == "__main__":
    main()
