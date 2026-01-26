"""Pose Tracker Teleoperation for Franka robot via ROS2

Subscribe to pose tracker and gripper commands from ROS2 topics
and teleoperate the Franka robot. Gripper data is collected but not controlled.

Usage:
    python examples/2_tracker_teleop.py [OPTIONS]

Options:
    --ip TEXT                 Robot server IP address [default: 192.168.1.7]
    --port INTEGER            Robot server port [default: 4242]
    --control-rate INTEGER    Control rate in Hz [default: 50]
    --translation-scale FLOAT Scale factor for translation [default: 1.0]
    --help                    Show this message and exit

Controls:
    - Move tracker: Control robot end-effector pose
    - Ctrl+C: Exit

ROS2 Topics:
    Subscribed:
        - tracker/ee_pose (geometry_msgs/Pose): Tracker end-effector pose (xyzw quaternion)
        - tracker/gripper (std_msgs/Float64): Gripper control value (0.0=open, 1.0=close) - data collection only

    Published:
        - robot/ee_pose (geometry_msgs/PoseStamped): Current robot end-effector pose (xyzw quaternion)
        - robot/joint_states (sensor_msgs/JointState): Current robot joint positions
        - robot/gripper (std_msgs/Float64): Current gripper command from tracker (not controlling robot gripper)

Quaternion Conventions:
    - Robot (Franka): xyzw format [qx, qy, qz, qw]
    - ROS2 geometry_msgs/Pose: xyzw format [x, y, z, w]
    - Internal storage: wxyz format for transforms3d compatibility

Examples:
    # Use default settings
    python examples/2_tracker_teleop.py

    # Connect to different robot IP
    python examples/2_tracker_teleop.py --ip 192.168.1.7

    # Adjust translation sensitivity
    python examples/2_tracker_teleop.py --translation-scale 0.5
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
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from dex_control.robot.robot_client import FrankaRobotClient


# Default CONFIG
DEFAULT_IP = "192.168.1.7"
DEFAULT_PORT = 4242
CONTROL_RATE_HZ = 50
TRANSLATION_SCALE = 1.0

# For safety, we limit the maximum delta of translation and rotation
# Feel free to adjust these values based on your setup
MAX_DELTA_TRANSLATION = 0.05  # Maximum position delta per step (m)
MAX_DELTA_ROTATION = 0.1      # Maximum rotation delta per step (rad)

# Quaternion conventions:
#   - Robot (Franka): xyzw format [qx, qy, qz, qw]
#   - ROS2 geometry_msgs/Pose: xyzw format [x, y, z, w]
#   - transforms3d (quat2mat, mat2quat): wxyz format [qw, qx, qy, qz]
#   - scipy Rotation: xyzw format [x, y, z, w]
# Internal storage uses wxyz for transforms3d compatibility


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))


def clamp_vector(vec: np.ndarray, max_val: float) -> np.ndarray:
    """Clamp each element of a vector to [-max_val, max_val]."""
    return np.clip(vec, -max_val, max_val)

#TODO: double check this based on our setup
def swap_y_z_axis(T: np.ndarray) -> np.ndarray:
    """Swap Y and Z axes in a 4x4 transformation matrix.
    
    Args:
        T: 4x4 transformation matrix
    
    Returns:
        New transformation matrix with Y and Z swapped
    """
    T_new = T.copy()
    
    # Swap rotation rows (Y and Z)
    T_new[1, :], T_new[2, :] = T[2, :].copy(), T[1, :].copy()
    
    # Swap rotation columns (Y and Z)
    T_new[:, 1], T_new[:, 2] = T_new[:, 2].copy(), T_new[:, 1].copy()
    
    return T_new


def rfu_to_flu(T_rfu: np.ndarray) -> np.ndarray:
    """Convert transformation matrix from RFU (Right, Front, Up) to FLU (Front, Left, Up).
    
    Args:
        T_rfu: 4x4 transformation matrix in RFU coordinates
    
    Returns:
        4x4 transformation matrix in FLU coordinates
    """
    C = np.array([
        [0,  1, 0, 0],
        [-1, 0, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    
    C_inv = C.T
    T_flu = C @ T_rfu @ C_inv
    
    return T_flu


class TrackerTeleopNode(Node):
    """ROS2 node for pose tracker teleoperation."""

    def __init__(
        self,
        robot: FrankaRobotClient,
        trans_scale: float = 1.0,
    ):
        super().__init__("tracker_teleop")

        self.logger = spdlog.ConsoleLogger("TrackerTeleop")
        self.logger.set_level(spdlog.LogLevel.INFO)

        self.robot = robot
        self.trans_scale = trans_scale
        self.gripper_control = 0.0  # Gripper data for collection (not controlling)
        self.lock = threading.Lock()

        # Initialize state variables
        self.stop_move = False
        self.end_robot = False
        self.reset_robot = False
        
        # Get initial arm pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        self.init_arm_ee_pose = self._get_tcp_position()
        # Build 4x4 transformation matrix from initial pose
        self.init_arm_ee_to_world = np.eye(4)
        self.init_arm_ee_to_world[:3, 3] = self.init_arm_ee_pose[:3]  # translation
        self.init_arm_ee_to_world[:3, :3] = quat2mat(self.init_arm_ee_pose[3:7])  # rotation (wxyz)
        
        # Tracker pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion for transforms3d)
        self.tracker_pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.tracker_pose_updated = False
        
        # Previous target pose for delta clamping: [x, y, z, qw, qx, qy, qz] (wxyz)
        self.prev_target_pose = self.init_arm_ee_pose.copy()

        # Set up ROS2 subscribers and publishers
        self._setup_ros_interfaces()
        
        self.logger.info("TrackerTeleopNode initialized")
        self.logger.info(f"Initial EE pose: {self.init_arm_ee_pose}")

    def _setup_ros_interfaces(self):
        """Set up all ROS2 subscribers and publishers."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # ============ Subscribers ============
        # Subscribe to tracker end-effector pose
        self.ee_pose_sub = self.create_subscription(
            Pose,
            "tracker/ee_pose",
            self._callback_ee_pose,
            qos_profile,
        )

        # Subscribe to tracker gripper control (data collection only, no control)
        self.gripper_sub = self.create_subscription(
            Float64,
            "tracker/gripper",
            self._callback_gripper,
            qos_profile,
        )

        # ============ Publishers ============
        # Publish current robot end-effector pose
        self.robot_ee_pose_pub = self.create_publisher(
            PoseStamped,
            "robot/ee_pose",
            qos_profile,
        )
        
        # Publish current robot joint states
        self.robot_joint_states_pub = self.create_publisher(
            JointState,
            "robot/joint_states",
            qos_profile,
        )

        # Publish gripper command from tracker (for data collection)
        self.robot_gripper_pub = self.create_publisher(
            Float64,
            "robot/gripper",
            qos_profile,
        )

        self.get_logger().info("Subscribed to tracker/ee_pose and tracker/gripper")
        self.get_logger().info("Publishing to robot/ee_pose, robot/joint_states, robot/gripper")

    def _get_tcp_position(self) -> np.ndarray:
        """Get the current TCP position from robot.
        
        Returns:
            Array of [x, y, z, qw, qx, qy, qz] (position + quaternion in wxyz for transforms3d)
        """
        pose = self.robot.get_ee_pose()
        t = pose["t"]  # [x, y, z]
        q = pose["q"]  # [qx, qy, qz, qw] from robot (xyzw format)
        
        # Convert from robot xyzw to internal wxyz format for transforms3d
        return np.array([t[0], t[1], t[2], q[3], q[0], q[1], q[2]])

    def _callback_ee_pose(self, pose: Pose):
        """Callback function to update tracker pose.
        
        Args:
            pose: Pose message containing tracker end-effector pose
                  ROS2 Pose uses xyzw quaternion format
            
        Note:
            This assumes the tracker pose is in left-hand coordinate system.
            Modify coordinate transformations if your tracker uses a different system.
        """
        if pose is None:
            return
            
        with self.lock:
            pos = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])
            
            # ROS2 Pose quaternion is xyzw, convert to wxyz for transforms3d
            quat_wxyz = [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ]
            
            rot = quat2mat(quat_wxyz)
            transmat = np.zeros((4, 4))
            transmat[:3, :3] = rot
            transmat[:3, 3] = pos
            transmat[3, 3] = 1.0
            
            # Apply coordinate transformations
            transmat = swap_y_z_axis(transmat)
            transmat = rfu_to_flu(transmat)
            
            rot = transmat[:3, :3]
            pos = transmat[:3, 3]
            rot_quat_wxyz = mat2quat(rot)  # Returns wxyz
            
            # Store as [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
            self.tracker_pose = np.concatenate([pos, rot_quat_wxyz], axis=0)
            self.tracker_pose_updated = True

    def _callback_gripper(self, data: Float64):
        """Callback function to update gripper control value (data collection only)."""
        with self.lock:
            self.gripper_control = data.data

    def get_gripper_control(self) -> float:
        """Get current gripper control value (thread-safe)."""
        with self.lock:
            return self.gripper_control

    def _retarget_base(self) -> np.ndarray:
        """Retarget the tracker pose to robot arm pose with delta clamping.
        
        Returns:
            Target pose as [x, y, z, qw, qx, qy, qz] (wxyz quaternion for transforms3d)
            Note: Caller must convert to robot xyzw format before sending to robot
        """
        with self.lock:
            # tracker_pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
            tracker_pose = self.tracker_pose.copy()
        
        # Calculate desired pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        desired_pose = self.init_arm_ee_pose.copy()
        
        # Apply translation with scaling: tracker_pos * scale + init_pos
        desired_pose[:3] = tracker_pose[:3] * self.trans_scale + self.init_arm_ee_to_world[:3, 3]
        
        # Apply rotation: tracker rotation combined with initial arm rotation
        # quat2mat expects wxyz format: tracker_pose[3:7] = [qw, qx, qy, qz]
        tracker_rot_mat = quat2mat(tracker_pose[3:7])
        combined_rot = tracker_rot_mat @ self.init_arm_ee_to_world[:3, :3]
        desired_pose[3:7] = mat2quat(combined_rot)  # mat2quat returns wxyz
        
        # Clamp translation delta for safety
        translation_delta = desired_pose[:3] - self.prev_target_pose[:3]
        clamped_translation_delta = clamp_vector(translation_delta, MAX_DELTA_TRANSLATION)
        
        # Clamp rotation delta (using rotation vector representation)
        # prev_target_pose[3:7] = [qw, qx, qy, qz] (wxyz)
        prev_rot = quat2mat(self.prev_target_pose[3:7])
        desired_rot = quat2mat(desired_pose[3:7])
        # Compute relative rotation: R_delta = R_desired @ R_prev^T
        delta_rot = desired_rot @ prev_rot.T
        # Convert to rotation vector for clamping (scipy uses xyzw internally)
        delta_rotvec = R.from_matrix(delta_rot).as_rotvec()
        clamped_rotvec = clamp_vector(delta_rotvec, MAX_DELTA_ROTATION)
        # Convert back to rotation matrix and apply to previous rotation
        clamped_delta_rot = R.from_rotvec(clamped_rotvec).as_matrix()
        clamped_rot = clamped_delta_rot @ prev_rot
        
        # Build clamped target pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        clamped_pose = self.prev_target_pose.copy()
        clamped_pose[:3] = self.prev_target_pose[:3] + clamped_translation_delta
        clamped_pose[3:7] = mat2quat(clamped_rot)  # mat2quat returns wxyz
        
        # Update previous target pose for next iteration
        self.prev_target_pose = clamped_pose.copy()
        
        return clamped_pose

    def is_pose_updated(self) -> bool:
        """Check if pose has been updated."""
        with self.lock:
            updated = self.tracker_pose_updated
            self.tracker_pose_updated = False
            return updated

    def publish_robot_state(self):
        """Publish current robot state to ROS2 topics.

        Publishes:
            - robot/ee_pose: Current end-effector pose (xyzw quaternion)
            - robot/joint_states: Current joint positions
            - robot/gripper: Current gripper command from tracker
        """
        timestamp = self.get_clock().now().to_msg()

        # Get current robot state
        try:
            ee_pose = self.robot.get_ee_pose()
            joint_positions = self.robot.get_joint_positions()
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot state: {e}")
            return

        # Publish end-effector pose (xyzw quaternion format)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = ee_pose["t"][0]
        pose_msg.pose.position.y = ee_pose["t"][1]
        pose_msg.pose.position.z = ee_pose["t"][2]
        # Robot returns [qx, qy, qz, qw] (xyzw), same as ROS2 Pose
        pose_msg.pose.orientation.x = ee_pose["q"][0]
        pose_msg.pose.orientation.y = ee_pose["q"][1]
        pose_msg.pose.orientation.z = ee_pose["q"][2]
        pose_msg.pose.orientation.w = ee_pose["q"][3]
        self.robot_ee_pose_pub.publish(pose_msg)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp
        joint_msg.name = [f"franka_joint_{i+1}" for i in range(7)]
        joint_msg.position = list(joint_positions)
        self.robot_joint_states_pub.publish(joint_msg)

        # Publish gripper command from tracker (for data collection)
        gripper_msg = Float64()
        gripper_msg.data = self.get_gripper_control()
        self.robot_gripper_pub.publish(gripper_msg)


def run_ros2_spin(node: Node, stop_event: threading.Event):
    """Run ROS2 spin in a separate thread."""
    while not stop_event.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--control-rate", default=CONTROL_RATE_HZ, help="Control rate in Hz")
@click.option("--translation-scale", default=TRANSLATION_SCALE, help="Scale factor for translation")
def main(ip, port, control_rate, translation_scale):
    """Pose Tracker Teleoperation for Franka robot via ROS2."""

    # Initialize ROS2
    rclpy.init()

    # Build server address
    server_addr = f"tcp://{ip}:{port}"

    # Connect to robot
    try:
        cprint(f"Connecting to robot at {server_addr}...", "cyan")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Robot connected!", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        rclpy.shutdown()
        return

    # Create teleop node
    teleop_node = TrackerTeleopNode(
        robot=robot,
        trans_scale=translation_scale,
    )

    # Start ROS2 spin in a separate thread
    stop_event = threading.Event()
    ros_thread = threading.Thread(
        target=run_ros2_spin, args=(teleop_node, stop_event), daemon=True
    )
    ros_thread.start()

    dt = 1.0 / control_rate

    # State tracking
    error_count = 0
    skip_commands = 0
    waiting_for_first_pose = True

    try:
        while rclpy.ok():
            # Wait for first pose
            if waiting_for_first_pose:
                if not teleop_node.is_pose_updated():
                    cprint("Waiting for tracker pose on tracker/ee_pose...", "yellow", end="\r")
                    time.sleep(0.1)
                    continue
                else:
                    cprint("\nTracker pose received! Starting teleoperation...           ", "green")
                    waiting_for_first_pose = False
                    time.sleep(0.5)
                    continue

            # Skip if in error recovery
            if skip_commands > 0:
                skip_commands -= 1
                time.sleep(dt)
                continue

            # Get retargeted pose (returns [x, y, z, qw, qx, qy, qz] in wxyz format)
            desired_pose = teleop_node._retarget_base()
            
            # Convert from internal wxyz to robot xyzw format [x,y,z,qx,qy,qz,qw]
            target_pose = [
                desired_pose[0],  # x
                desired_pose[1],  # y
                desired_pose[2],  # z
                desired_pose[4],  # qx (from wxyz[1])
                desired_pose[5],  # qy (from wxyz[2])
                desired_pose[6],  # qz (from wxyz[3])
                desired_pose[3],  # qw (from wxyz[0])
            ]

            # Send pose command to robot
            try:
                robot.move_ee_pose(
                    target_pose,
                    asynchronous=True,
                    delta=False,  # Absolute pose
                )
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

            # Publish current robot state for data collection
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
