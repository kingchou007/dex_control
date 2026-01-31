"""Client for Franka Robot Control Server.

Author: Jinzhou Li

This client provides a Python interface to control the Franka robot remotely via RPC.
It uses zerorpc for communication with the robot server and includes automatic retry
and reconnection logic for robust operation.

Usage:
    from dex_control.robot.robot_client import FrankaRobotClient
    
    robot = FrankaRobotClient(server_addr="tcp://192.168.1.7:4242")
    pose = robot.get_ee_pose()
    robot.move_ee_pose([0.0, 0.0, 0.01], delta=True)  # Move down 1cm
"""

import time
import functools
from typing import List, Dict, Any
import numpy as np
import zerorpc
import click
import spdlog

logger = spdlog.ConsoleLogger("FrankaRobotClient")
logger.set_level(spdlog.LogLevel.INFO)


def robust_call(max_retries: int = 3, reconnect: bool = True):
    """Decorator to automatically retry and reconnect failed zerorpc calls.

    Args:
        max_retries: Maximum number of retry attempts (default: 3).
        reconnect: Whether to attempt reconnecting after each failure (default: True).

    Returns:
        Decorated function with retry and reconnection logic.

    See also: 
        https://github.com/willjhliang/eva/blob/main/eva/robot/server_interface.py
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            last_err = None
            for i in range(max_retries):
                try:
                    return func(self, *args, **kwargs)
                except zerorpc.exceptions.RemoteError as e:
                    last_err = e
                    logger.warn(f"RemoteError in {func.__name__}, attempt {i+1}/{max_retries}: {e}")
                except zerorpc.exceptions.TimeoutExpired as e:
                    last_err = e
                    logger.warn(f"Timeout in {func.__name__}, attempt {i+1}/{max_retries}: {e}")
                except Exception as e:
                    last_err = e
                    logger.error(f"Unexpected error in {func.__name__}, attempt {i+1}/{max_retries}: {e}")

                if reconnect:
                    try:
                        logger.info("Reconnecting RPC client...")
                        self._connect()
                    except Exception as e:
                        logger.error(f"Reconnect failed: {e}")

                time.sleep(0.5)

            logger.error(f"All {max_retries} attempts failed for {func.__name__}, raising last error.")
            if last_err is not None:
                raise last_err
            return func(self, *args, **kwargs)
        return wrapper
    return decorator


class FrankaRobotClient:
    """Robot client for controlling the Franka robot remotely via RPC.
    
    This client provides a high-level interface to the Franka robot control server,
    with automatic retry and reconnection logic for robust operation.
    """

    def __init__(self, server_addr: str = "tcp://192.168.1.7:4242", franka_hand: bool = False):
        """Initialize the FrankaRobotClient.

        Args:
            server_addr: RPC server address (e.g., "tcp://192.168.1.7:4242").
            franka_hand: Whether using Franka Hand gripper (currently unused).
        """
        self.server_addr = server_addr
        self.client: zerorpc.Client = None
        self._connect()

    def _connect(self):
        """Establish connection to the Franka RPC server."""
        self.client = zerorpc.Client(heartbeat=20, timeout=60)
        self.client.connect(self.server_addr)
        logger.info(f"Connected to robot server at {self.server_addr}")
        logger.info(f"--------------------------------")
        # add empy space between the lines
        logger.info(f"")
        logger.info(f"")

    @robust_call()
    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions.

        Returns:
            Array of 7 joint angles in radians.
        """
        return np.array(self.client.get_joint_positions())

    @robust_call()
    def get_ee_pose(self) -> Dict[str, np.ndarray]:
        """Get current end-effector pose.

        Returns:
            Dictionary with keys:
                - 't': translation [x, y, z] in meters
                - 'q': quaternion [qx, qy, qz, qw]
        """
        pose = self.client.get_ee_pose()
        return {
            "t": np.array(pose["t"]),
            "q": np.array(pose["q"])
        }

    @robust_call()
    def get_external_torques(self) -> np.ndarray:
        """Get external torques on each joint.

        Returns:
            Array of 7 external torques (one per joint) in Nm.
        """
        return np.array(self.client.get_external_torques())

    @robust_call()
    def get_state(self) -> Dict[str, Any]:
        """Get full robot state.
        
        Note: Currently not implemented on server side.
        """
        raise NotImplementedError("get_state is not implemented")

    @robust_call()
    def move_ee_pose(
        self,
        target_ee_pose: List[float],
        asynchronous: bool = True,
        delta: bool = True
    ) -> None:
        """Move end-effector to target pose.

        Args:
            target_ee_pose: Target pose as:
                - [x, y, z]: translation only (3D)
                - [qx, qy, qz, qw]: rotation only (4D)
                - [x, y, z, qx, qy, qz, qw]: full pose (7D)
            asynchronous: If True, return immediately without waiting.
            delta: If True, interpret pose as relative change.
        """
        return self.client.move_ee_pose(target_ee_pose, asynchronous, delta)

    @robust_call()
    def move_ee_waypoints(self, waypoints: List[List[float]], delta: bool = True) -> None:
        """Move end-effector through a sequence of waypoints.

        Args:
            waypoints: List of target poses (each 3D, 4D, or 7D).
            delta: If True, interpret waypoints as relative changes.
        """
        return self.client.move_ee_waypoints(waypoints, delta)

    @robust_call()
    def move_joints(
        self,
        target_joint_positions: List[float],
        asynchronous: bool = False
    ) -> None:
        """Move joints to target configuration.

        Args:
            target_joint_positions: Target joint angles (7 values in radians).
            asynchronous: If True, return immediately without waiting.
        """
        return self.client.move_joints(target_joint_positions, asynchronous)

    @robust_call()
    def move_joint_waypoints(self, waypoints: List[List[float]]) -> None:
        """Move joints through a sequence of waypoints.

        Args:
            waypoints: List of joint configurations (each with 7 angles).
        """
        return self.client.move_joint_waypoints(waypoints)

    # ------------------------------------------------------------------
    # Gripper State (New)
    # ------------------------------------------------------------------

    @robust_call()
    def get_gripper_width(self) -> float:
        """Get current gripper width in meters."""
        return float(self.client.get_gripper_width())

    @robust_call()
    def get_gripper_max_width(self) -> float:
        """Get maximum gripper width in meters."""
        return float(self.client.get_gripper_max_width())

    @robust_call()
    def is_gripper_grasped(self) -> bool:
        """Check if gripper is currently holding an object."""
        return bool(self.client.is_gripper_grasped())

    # ------------------------------------------------------------------
    # Gripper Control (Updated)
    # ------------------------------------------------------------------
    @robust_call()
    def move_gripper(self, target_width: float, speed: float = 0.1, asynchronous: bool = False) -> bool:
        """Move gripper to specific width.

        Args:
            target_width: Target width in meters.
            speed: Speed in m/s (default: 0.1).
            asynchronous: If True, return immediately without waiting for completion.
        """
        return self.client.move_gripper(target_width, speed, asynchronous)

    @robust_call()
    def open_gripper(self, speed: float = 0.1, asynchronous: bool = False) -> bool:
        """Fully open the gripper.

        Args:
            speed: Speed in m/s.
            asynchronous: If True, return immediately.
        """
        return self.client.open_gripper(speed, asynchronous)
    
    def release_object(self) -> bool:
        """Alias for open_gripper (for backward compatibility)."""
        return self.open_gripper()

    @robust_call()
    def grasp_object(
        self, 
        width: float = 0.0, 
        speed: float = 0.1, 
        force: float = 60.0, 
        epsilon_inner: float = 0.005, 
        epsilon_outer: float = 0.005,
        asynchronous: bool = False
    ) -> bool:
        """Grasp an object.

        Args:
            width: Expected object width (m). If 0, attempts to grasp at closed position.
            speed: Closing speed (m/s).
            force: Grasping force (N).
            epsilon_inner: Tolerance for object being smaller than expected.
            epsilon_outer: Tolerance for object being larger than expected.
            asynchronous: If True, return immediately.
        """
        return self.client.grasp_object(width, speed, force, epsilon_inner, epsilon_outer, asynchronous)

    @robust_call()
    def stop_gripper(self, asynchronous: bool = False) -> bool:
        """Immediately stop gripper motion."""
        return self.client.stop_gripper(asynchronous)

    @robust_call()
    def homing_gripper(self, asynchronous: bool = False) -> bool:
        """Calibrate gripper (Homing). usually done once after startup."""
        return self.client.homing_gripper(asynchronous)

    # ------------------------------------------------------------------
    # Impedance Control / Kinesthetic Teaching
    # ------------------------------------------------------------------
    @robust_call()
    def set_joint_impedance(self, impedance: List[float]) -> bool:
        """Set joint impedance values.

        Args:
            impedance: List of 7 impedance values (one per joint).
                      Lower values = more compliant/backdrivable.
                      Set all to 0 for pure gravity compensation.

        Returns:
            bool: True if successful.
        """
        return self.client.set_joint_impedance(impedance)

    @robust_call()
    def start_gravity_compensation(self, duration: float = 3600.0) -> bool:
        """Start gravity compensation mode for kinesthetic teaching.

        Uses JointVelocityMotion with zero velocities and zero impedance
        to allow manual manipulation of the robot.

        Args:
            duration: Duration in seconds (default: 1 hour).

        Returns:
            bool: True if motion started successfully.
        """
        return self.client.start_gravity_compensation(duration)

    @robust_call()
    def stop_motion(self) -> bool:
        """Stop any ongoing motion."""
        return self.client.stop_motion()

    def close(self):
        """Close the RPC connection."""
        if self.client:
            self.client.close()


@click.command()
@click.option("--ip", default="192.168.1.7", help="Robot server IP address")
@click.option("--port", default=4242, help="Robot server port")
def main(ip, port):
    """Franka Robot Client CLI - Test connection and get robot state."""
    try:
        robot = FrankaRobotClient(f"tcp://{ip}:{port}")
        
        logger.info("Testing robot connection...")
        
        # Get and display external torques
        for i in range(100):
            external_torques = robot.get_external_torques()
            logger.info(f"External Torques: {external_torques}")
            time.sleep(0.1)
        
        logger.info("Connection test successful!")
        robot.close()
    except Exception as e:
        logger.error(f"Connection test failed: {e}")


if __name__ == "__main__":
    main()