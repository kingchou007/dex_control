"""Pose Tracker Teleoperation for Franka robot via ZMQ

Subscribes to tracker pose via ZMQ and teleoperates the Franka robot.
Publishes robot state via ZMQ for data collection.

Usage:
    python examples/2_tracker_teleop.py [OPTIONS]

The tracker source (e.g. vive_tracker) should publish pose as JSON via ZMQ PUB:
    {"position": [x, y, z], "orientation": [qx, qy, qz, qw]}
"""

import threading
import time
import json
import numpy as np
import click
import zmq
import spdlog
from termcolor import cprint
from transforms3d.quaternions import quat2mat, mat2quat
from scipy.spatial.transform import Rotation as R

from dex_control.robot.robot_client import FrankaRobotClient
from dex_control.utils.zmq_publisher import ZMQStatePublisher


# Default CONFIG
DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242
CONTROL_RATE_HZ = 50
TRANSLATION_SCALE = 1.0

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


class TrackerZMQReceiver:
    """Receives tracker pose via ZMQ SUB in a background thread."""

    def __init__(self, host: str = "localhost", port: int = 5558):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.RCVTIMEO, 100)

        self.lock = threading.Lock()
        self.latest_pose = None  # {"position": [x,y,z], "orientation": [qx,qy,qz,qw]}
        self.pose_updated = False
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

    def _recv_loop(self):
        while self.running:
            try:
                msg = self.socket.recv_string()
                data = json.loads(msg)
                with self.lock:
                    self.latest_pose = data
                    self.pose_updated = True
            except zmq.Again:
                continue

    def get_pose(self):
        """Returns (position, orientation_xyzw) or (None, None)."""
        with self.lock:
            if self.latest_pose is None:
                return None, None
            pos = self.latest_pose["position"]
            quat = self.latest_pose["orientation"]
            return pos, quat

    def is_updated(self):
        with self.lock:
            updated = self.pose_updated
            self.pose_updated = False
            return updated

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.socket.close()
        self.context.term()


class TrackerTeleop:
    """Tracker teleoperation controller."""

    def __init__(self, robot: FrankaRobotClient, trans_scale: float = 1.0):
        self.logger = spdlog.ConsoleLogger("TrackerTeleop")
        self.logger.set_level(spdlog.LogLevel.INFO)

        self.robot = robot
        self.trans_scale = trans_scale
        self.signal_processor = ViveSignalProcessor()

        # Get initial arm pose: [x, y, z, qw, qx, qy, qz] (wxyz quaternion)
        self.init_arm_ee_pose = self._get_tcp_position()

        # Build 4x4 transformation matrix from initial pose
        self.init_arm_ee_to_world = np.eye(4)
        self.init_arm_ee_to_world[:3, 3] = self.init_arm_ee_pose[:3]
        self.init_arm_ee_to_world[:3, :3] = quat2mat(self.init_arm_ee_pose[3:7])

        # Previous target pose for delta clamping
        self.prev_target_pose = self.init_arm_ee_pose.copy()

        self.logger.info("TrackerTeleop initialized")
        self.logger.info(f"Initial EE pose: {self.init_arm_ee_pose}")

    def _get_tcp_position(self) -> np.ndarray:
        """Get the current TCP position from robot (Internal wxyz format)."""
        pose = self.robot.get_ee_pose()
        t = pose["t"]  # [x, y, z]
        q = pose["q"]  # [qx, qy, qz, qw] from robot

        # Convert to internal [x, y, z, w, x, y, z]
        return np.array([t[0], t[1], t[2], q[3], q[0], q[1], q[2]])

    def update(self, pos, quat_xyzw) -> np.ndarray:
        """Process tracker pose and return retargeted robot pose."""
        rel_pos, rel_quat_wxyz = self.signal_processor.process(pos, quat_xyzw)
        tracker_pose = np.concatenate([rel_pos, rel_quat_wxyz], axis=0)
        return self._retarget(tracker_pose)

    def _retarget(self, tracker_pose) -> np.ndarray:
        """Retarget the tracker pose to robot arm pose with delta clamping."""
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


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--control-rate", default=CONTROL_RATE_HZ, help="Control rate in Hz")
@click.option("--translation-scale", default=TRANSLATION_SCALE, help="Scale factor for translation")
@click.option("--tracker-host", default="localhost", help="ZMQ host for tracker pose")
@click.option("--tracker-port", default=5558, help="ZMQ SUB port for tracker pose")
@click.option("--zmq-port", default=5557, help="ZMQ PUB port for state streaming")
def main(ip, port, control_rate, translation_scale, tracker_host, tracker_port, zmq_port):
    """Pose Tracker Teleoperation for Franka robot via ZMQ."""

    server_addr = f"tcp://{ip}:{port}"

    try:
        cprint(f"Connecting to robot at {server_addr}...", "cyan")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Robot connected!", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        return

    # Initialize ZMQ
    zmq_pub = ZMQStatePublisher(port=zmq_port)
    cprint(f"ZMQ state publisher on port {zmq_port}", "green")

    tracker_recv = TrackerZMQReceiver(host=tracker_host, port=tracker_port)
    cprint(f"ZMQ tracker subscriber on {tracker_host}:{tracker_port}", "green")

    teleop = TrackerTeleop(robot=robot, trans_scale=translation_scale)

    dt = 1.0 / control_rate
    error_count = 0
    skip_commands = 0
    waiting_for_first_pose = True
    robot.grasp_object()

    try:
        while True:
            if waiting_for_first_pose:
                if not tracker_recv.is_updated():
                    cprint("Waiting for tracker pose...", "yellow", end="\r")
                    time.sleep(0.1)
                    continue
                else:
                    cprint("\nTracker found! Starting teleop...       ", "green")
                    waiting_for_first_pose = False
                    time.sleep(0.5)
                    continue

            if skip_commands > 0:
                skip_commands -= 1
                time.sleep(dt)
                continue

            # Get tracker pose
            pos, quat_xyzw = tracker_recv.get_pose()
            if pos is None:
                time.sleep(dt)
                continue

            # Retarget to robot pose (wxyz format)
            desired_pose = teleop.update(pos, quat_xyzw)

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

            # Publish state via ZMQ
            try:
                ee_pose = robot.get_ee_pose()
                joints = robot.get_joint_positions()
                try:
                    gripper_width = robot.get_gripper_width()
                except Exception:
                    gripper_width = None
                zmq_pub.publish(ee_pose, joints, gripper_width)
            except Exception:
                pass

            time.sleep(dt)

    except KeyboardInterrupt:
        cprint("\n\nShutting down...", "yellow")
    finally:
        tracker_recv.close()
        zmq_pub.close()


if __name__ == "__main__":
    main()
