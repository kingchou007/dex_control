"""Pose Tracker Teleoperation for Franka robot via ROS2 (TF Version)

Subscribe to /tf topic (specifically for 'vive_tracker' frame) 
and teleoperate the Franka robot.

Usage:
    python examples/2_tracker_teleop.py [OPTIONS]
"""

import threading
import time
import numpy as np
import click
import spdlog
from termcolor import cprint
from transforms3d.quaternions import quat2mat, mat2quat
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage  # Import TFMessage

from dex_control.robot.robot_client import FrankaRobotClient


# Default CONFIG
DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242
CONTROL_RATE_HZ = 50
TRANSLATION_SCALE = 1.0
DEFAULT_TARGET_FRAME = "vive_tracker"  # Default frame from your log

# Safety Limits
MAX_DELTA_TRANSLATION = 0.5  # Maximum position delta per step (m)
MAX_DELTA_ROTATION = 0.5      # Maximum rotation delta per step (rad)


def clamp_vector(vec: np.ndarray, max_val: float) -> np.ndarray:
    """Clamp each element of a vector to [-max_val, max_val]."""
    return np.clip(vec, -max_val, max_val)


# =========================================================================
# ViveSignalProcessor: Please modify this based on init tracker pose
# =========================================================================
class ViveSignalProcessor:
    def __init__(self):
        self.init_pos = None
        self.init_rot_inv = None
        self._C = np.array([[1,  0,  0],
                           [0,  0, 1],
                           [0, -1,  0]], dtype=float)

    def process(self, pos, quat_xyzw):
        """
        Raw: pos=[x,y,z], quat=[x,y,z,w]
        Output pos=[x,y,z], quat=[w,x,y,z]
        """
        pos_transformed = self._C @ np.array(pos)
        rot_mat = R.from_quat(quat_xyzw).as_matrix()
        rot_transformed = self._C @ rot_mat @ self._C.T
        curr_rot = R.from_matrix(rot_transformed)

        if self.init_pos is None:
            self.init_pos = pos_transformed.copy()
            self.init_rot_inv = curr_rot.inv()
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])

        rel_pos = pos_transformed - self.init_pos
        rel_rot = self.init_rot_inv * curr_rot
        qx, qy, qz, qw = rel_rot.as_quat()
        
        return rel_pos, np.array([qw, qx, qy, qz])

class TrackerTeleopNode(Node):
    """ROS2 node for pose tracker teleoperation via TF."""

    def __init__(
        self,
        robot: FrankaRobotClient,
        trans_scale: float = 1.0,
        target_frame: str = "vive_tracker"
    ):
        super().__init__("tracker_teleop")

        self.logger = spdlog.ConsoleLogger("TrackerTeleop")
        self.logger.set_level(spdlog.LogLevel.INFO)

        self.robot = robot
        self.trans_scale = trans_scale
        self.target_frame = target_frame
        self.lock = threading.Lock()
        self.signal_processor = ViveSignalProcessor()

        # Get initial arm pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        self.init_arm_ee_pose = self._get_tcp_position()
        
        # Build 4x4 transformation matrix from initial pose
        self.init_arm_ee_to_world = np.eye(4)
        self.init_arm_ee_to_world[:3, 3] = self.init_arm_ee_pose[:3]
        self.init_arm_ee_to_world[:3, :3] = quat2mat(self.init_arm_ee_pose[3:7])
        
        # Tracker pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        self.tracker_pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.tracker_pose_updated = False
        
        # Previous target pose for delta clamping
        self.prev_target_pose = self.init_arm_ee_pose.copy()

        # Set up ROS2 subscribers and publishers
        self._setup_ros_interfaces()
        
        self.logger.info("TrackerTeleopNode initialized")
        self.logger.info(f"Tracking TF frame: {self.target_frame}")
        self.logger.info(f"Initial EE pose: {self.init_arm_ee_pose}")

    def _setup_ros_interfaces(self):
        """Set up all ROS2 subscribers and publishers."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # ============ Subscribers ============
        # Subscribe to /tf topic
        self.tf_sub = self.create_subscription(
            TFMessage,
            "/tf",
            self._callback_tf,
            qos_profile,
        )

        # ============ Publishers ============
        self.robot_ee_pose_pub = self.create_publisher(
            PoseStamped,
            "robot/ee_pose",
            qos_profile,
        )
        
        self.robot_joint_states_pub = self.create_publisher(
            JointState,
            "robot/joint_states",
            qos_profile,
        )

        self.get_logger().info(f"Subscribed to /tf (looking for '{self.target_frame}')")
        self.get_logger().info("Publishing to robot/ee_pose, robot/joint_states")

    def _get_tcp_position(self) -> np.ndarray:
        """Get the current TCP position from robot (Internal wxyz format)."""
        pose = self.robot.get_ee_pose()
        t = pose["t"]  # [x, y, z]
        q = pose["q"]  # [qx, qy, qz, qw] from robot
        
        # Convert to internal [x, y, z, w, x, y, z]
        return np.array([t[0], t[1], t[2], q[3], q[0], q[1], q[2]])

    def _callback_tf(self, msg: TFMessage):
        """Callback function to process /tf messages."""
        if msg is None:
            return

        # Iterate through all transforms in the message
        found = False
        target_transform = None

        for t in msg.transforms:
            if t.child_frame_id == self.target_frame:
                target_transform = t.transform
                found = True
                break
        
        if not found:
            return

        with self.lock:
            raw_pos = [
                target_transform.translation.x,
                target_transform.translation.y,
                target_transform.translation.z
            ]
            raw_quat_xyzw = [
                target_transform.rotation.x,
                target_transform.rotation.y,
                target_transform.rotation.z,
                target_transform.rotation.w
            ]
            
            rel_pos, rel_quat_wxyz = self.signal_processor.process(raw_pos, raw_quat_xyzw)
            self.tracker_pose = np.concatenate([rel_pos, rel_quat_wxyz], axis=0)
            self.tracker_pose_updated = True

    def _retarget_base(self) -> np.ndarray:
        """Retarget the tracker pose to robot arm pose with delta clamping."""
        with self.lock:
            tracker_pose = self.tracker_pose.copy()
        
        desired_pose = self.init_arm_ee_pose.copy()
        desired_pose[:3] = tracker_pose[:3] * self.trans_scale + self.init_arm_ee_to_world[:3, 3]
        
        tracker_rot_mat = quat2mat(tracker_pose[3:7])
        combined_rot = tracker_rot_mat @ self.init_arm_ee_to_world[:3, :3]
        desired_pose[3:7] = mat2quat(combined_rot)
        
        # Clamp translation delta
        translation_delta = desired_pose[:3] - self.prev_target_pose[:3]
        clamped_translation_delta = clamp_vector(translation_delta, MAX_DELTA_TRANSLATION)
        
        # Clamp rotation delta
        prev_rot = quat2mat(self.prev_target_pose[3:7])
        desired_rot = quat2mat(desired_pose[3:7])
        delta_rot = desired_rot @ prev_rot.T
        delta_rotvec = R.from_matrix(delta_rot).as_rotvec()
        clamped_rotvec = clamp_vector(delta_rotvec, MAX_DELTA_ROTATION)
        clamped_delta_rot = R.from_rotvec(clamped_rotvec).as_matrix()
        clamped_rot = clamped_delta_rot @ prev_rot
        
        # Build clamped target pose
        clamped_pose = self.prev_target_pose.copy()
        clamped_pose[:3] = self.prev_target_pose[:3] + clamped_translation_delta
        clamped_pose[3:7] = mat2quat(clamped_rot)
        
        # Update previous target pose
        self.prev_target_pose = clamped_pose.copy()
        
        return clamped_pose

    def is_pose_updated(self) -> bool:
        """Check if pose has been updated."""
        with self.lock:
            updated = self.tracker_pose_updated
            self.tracker_pose_updated = False
            return updated

    def publish_robot_state(self):
        """Publish current robot state to ROS2."""
        timestamp = self.get_clock().now().to_msg()

        try:
            ee_pose = self.robot.get_ee_pose()
            joint_positions = self.robot.get_joint_positions()
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot state: {e}")
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = ee_pose["t"][0]
        pose_msg.pose.position.y = ee_pose["t"][1]
        pose_msg.pose.position.z = ee_pose["t"][2]
        pose_msg.pose.orientation.x = ee_pose["q"][0]
        pose_msg.pose.orientation.y = ee_pose["q"][1]
        pose_msg.pose.orientation.z = ee_pose["q"][2]
        pose_msg.pose.orientation.w = ee_pose["q"][3]
        self.robot_ee_pose_pub.publish(pose_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = timestamp
        joint_msg.name = [f"franka_joint_{i+1}" for i in range(7)]
        joint_msg.position = list(joint_positions)
        self.robot_joint_states_pub.publish(joint_msg)


def run_ros2_spin(node: Node, stop_event: threading.Event):
    while not stop_event.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--control-rate", default=CONTROL_RATE_HZ, help="Control rate in Hz")
@click.option("--translation-scale", default=TRANSLATION_SCALE, help="Scale factor for translation")
@click.option("--target-frame", default=DEFAULT_TARGET_FRAME, help="Target TF frame ID to track")
def main(ip, port, control_rate, translation_scale, target_frame):
    """Pose Tracker Teleoperation for Franka robot via ROS2 (TF)."""

    rclpy.init()
    server_addr = f"tcp://{ip}:{port}"

    try:
        cprint(f"Connecting to robot at {server_addr}...", "cyan")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Robot connected!", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        rclpy.shutdown()
        return

    teleop_node = TrackerTeleopNode(
        robot=robot,
        trans_scale=translation_scale,
        target_frame=target_frame
    )

    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=run_ros2_spin, args=(teleop_node, stop_event), daemon=True
    )
    ros_thread.start()

    dt = 1.0 / control_rate
    error_count = 0
    skip_commands = 0
    waiting_for_first_pose = True
    robot.grasp_object()
    try:
        while rclpy.ok():
            if waiting_for_first_pose:
                if not teleop_node.is_pose_updated():
                    cprint(f"Waiting for TF frame '{target_frame}'...", "yellow", end="\r")
                    time.sleep(0.1)
                    continue
                else:
                    cprint(f"\nTarget frame '{target_frame}' found! Starting teleop...       ", "green")
                    waiting_for_first_pose = False
                    time.sleep(0.5)
                    continue

            if skip_commands > 0:
                skip_commands -= 1
                time.sleep(dt)
                continue

            # Get retargeted pose (wxyz format)
            desired_pose = teleop_node._retarget_base()
            
            # Convert to Robot format (xyzw)
            target_pose = [
                desired_pose[0],  # x
                desired_pose[1],  # y
                desired_pose[2],  # z
                desired_pose[4],  # qx
                desired_pose[5],  # qy
                desired_pose[6],  # qz
                desired_pose[3],  # qw
            ]

            try:
                robot.move_ee_pose(target_pose, asynchronous=True, delta=False)
                if error_count > 0:
                    error_count -= 1
            except Exception as e:
                error_msg = str(e)
                if "Reflex" in error_msg or "aborted" in error_msg:
                    error_count += 1
                    if error_count == 1:
                        cprint("\nRobot error - pausing...", "yellow")
                    skip_commands = 25
                    time.sleep(1.0)
                else:
                    cprint(f"Error: {e}", "red")

            teleop_node.publish_robot_state()
            time.sleep(dt)

    except KeyboardInterrupt:
        cprint("\n\nShutting down...", "yellow")
    finally:
        stop_event.set()
        ros_thread.join(timeout=1.0)
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()