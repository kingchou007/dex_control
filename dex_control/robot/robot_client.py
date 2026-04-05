"""Client for Franka Robot Control Server.

Author: Jinzhou Li

This client provides a Python interface to control the Franka robot remotely via gRPC.
Includes automatic retry and reconnection logic for robust operation.

Usage:
    from dex_control.robot.robot_client import FrankaRobotClient

    robot = FrankaRobotClient(server_addr="192.168.1.7:4242")
    pose = robot.get_ee_pose()
    robot.move_ee_pose([0.0, 0.0, 0.01], delta=True)  # Move down 1cm

See also:
    https://github.com/willjhliang/eva/blob/main/eva/robot/server_interface.py
"""

import time
from typing import List, Dict, Any
import numpy as np
import click
import grpc
import spdlog

from dex_control.proto import franka_pb2, franka_pb2_grpc

logger = spdlog.ConsoleLogger("FrankaRobotClient")
logger.set_level(spdlog.LogLevel.INFO)

_EMPTY = franka_pb2.Empty()


class FrankaRobotClient:
    """Robot client for controlling the Franka robot remotely via gRPC.

    All RPC calls are automatically retried with exponential backoff.
    """

    def __init__(self, server_addr: str = "192.168.1.7:4242", max_retries: int = 3):
        """Initialize the FrankaRobotClient.

        Args:
            server_addr: gRPC server address (e.g., "192.168.1.7:4242").
            max_retries: Max retry attempts for each RPC call (default: 3).
        """
        self.server_addr = server_addr
        self.max_retries = max_retries
        self.channel: grpc.Channel = None
        self.stub: franka_pb2_grpc.FrankaRobotStub = None
        self._connect()

    def _connect(self):
        """(Re)establish gRPC channel."""
        if self.channel is not None:
            self.channel.close()
        self.channel = grpc.insecure_channel(self.server_addr)
        self.stub = franka_pb2_grpc.FrankaRobotStub(self.channel)
        logger.info(f"Connected to robot server at {self.server_addr}")

    def _call(self, method: str, request=None):
        """Call a gRPC method with automatic retry and reconnection.

        Args:
            method: Name of the stub method to call.
            request: Protobuf request message (defaults to Empty).
        """
        if request is None:
            request = _EMPTY
        last_err = None
        for i in range(self.max_retries):
            try:
                return getattr(self.stub, method)(request)
            except grpc.RpcError as e:
                last_err = e
                logger.warn(f"gRPC error in {method}, attempt {i+1}/{self.max_retries}: {e}")

            is_last = i == self.max_retries - 1
            if not is_last:
                try:
                    logger.info("Reconnecting gRPC channel...")
                    self._connect()
                except Exception as e:
                    logger.error(f"Reconnect failed: {e}")
                time.sleep(0.5 * (2 ** i))

        raise last_err  # type: ignore[misc]

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        """Get current joint positions (7 joint angles in radians)."""
        resp = self._call("GetJointPositions")
        return np.array(resp.values)

    def get_ee_pose(self) -> Dict[str, np.ndarray]:
        """Get current end-effector pose.

        Returns:
            Dictionary with 't' (translation [x,y,z]) and 'q' (quaternion [qx,qy,qz,qw]).
        """
        resp = self._call("GetEePose")
        return {"t": np.array(resp.translation), "q": np.array(resp.quaternion)}

    def get_external_torques(self) -> np.ndarray:
        """Get external torques on each joint (7 values in Nm)."""
        resp = self._call("GetExternalTorques")
        return np.array(resp.values)

    def get_state(self) -> Dict[str, Any]:
        """Get full robot state."""
        resp = self._call("GetState")
        return {k: list(v.values) for k, v in resp.data.items()}

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_ee_pose(
        self,
        target_ee_pose: List[float],
        asynchronous: bool = True,
        delta: bool = True,
    ) -> bool:
        """Move end-effector to target pose.

        Args:
            target_ee_pose: [x,y,z] | [qx,qy,qz,qw] | [x,y,z,qx,qy,qz,qw].
            asynchronous: If True, return immediately without waiting.
            delta: If True, interpret pose as relative change.
        """
        req = franka_pb2.MoveEePoseRequest(
            target_pose=target_ee_pose, asynchronous=asynchronous, delta=delta,
        )
        return self._call("MoveEePose", req).success

    def move_ee_waypoints(self, waypoints: List[List[float]], delta: bool = True) -> bool:
        """Move end-effector through a sequence of waypoints."""
        wps = [franka_pb2.DoubleList(values=wp) for wp in waypoints]
        req = franka_pb2.WaypointsRequest(waypoints=wps, delta=delta)
        return self._call("MoveEeWaypoints", req).success

    def move_joints(
        self,
        target_joint_positions: List[float],
        asynchronous: bool = False,
    ) -> bool:
        """Move joints to target configuration (7 angles in radians)."""
        req = franka_pb2.MoveJointsRequest(
            target_positions=target_joint_positions, asynchronous=asynchronous,
        )
        return self._call("MoveJoints", req).success

    def move_joint_waypoints(self, waypoints: List[List[float]]) -> bool:
        """Move joints through a sequence of waypoints."""
        wps = [franka_pb2.DoubleList(values=wp) for wp in waypoints]
        req = franka_pb2.WaypointsRequest(waypoints=wps)
        return self._call("MoveJointWaypoints", req).success

    # ------------------------------------------------------------------
    # Gripper State
    # ------------------------------------------------------------------

    def get_gripper_width(self) -> float:
        """Get current gripper width in meters."""
        return self._call("GetGripperWidth").value

    def get_gripper_max_width(self) -> float:
        """Get maximum gripper width in meters."""
        return self._call("GetGripperMaxWidth").value

    def is_gripper_grasped(self) -> bool:
        """Check if gripper is currently holding an object."""
        return self._call("IsGripperGrasped").success

    # ------------------------------------------------------------------
    # Gripper Control
    # ------------------------------------------------------------------

    def move_gripper(self, target_width: float, speed: float = 0.1, asynchronous: bool = False) -> bool:
        """Move gripper to specific width (meters)."""
        req = franka_pb2.MoveGripperRequest(
            target_width=target_width, speed=speed, asynchronous=asynchronous,
        )
        return self._call("MoveGripper", req).success

    def open_gripper(self, speed: float = 0.1, asynchronous: bool = False) -> bool:
        """Fully open the gripper."""
        req = franka_pb2.GripperActionRequest(speed=speed, asynchronous=asynchronous)
        return self._call("OpenGripper", req).success

    def release_object(self) -> bool:
        """Alias for open_gripper."""
        return self.open_gripper()

    def grasp_object(
        self,
        width: float = 0.0,
        speed: float = 0.1,
        force: float = 60.0,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
        asynchronous: bool = False,
    ) -> bool:
        """Grasp an object.

        Args:
            width: Expected object width (m). 0 = grasp at closed position.
            speed: Closing speed (m/s).
            force: Grasping force (N).
            epsilon_inner: Tolerance for object smaller than expected.
            epsilon_outer: Tolerance for object larger than expected.
            asynchronous: If True, return immediately.
        """
        req = franka_pb2.GraspRequest(
            width=width, speed=speed, force=force,
            epsilon_inner=epsilon_inner, epsilon_outer=epsilon_outer,
            asynchronous=asynchronous,
        )
        return self._call("GraspObject", req).success

    def stop_gripper(self, asynchronous: bool = False) -> bool:
        """Immediately stop gripper motion."""
        req = franka_pb2.GripperActionRequest(asynchronous=asynchronous)
        return self._call("StopGripper", req).success

    def homing_gripper(self, asynchronous: bool = False) -> bool:
        """Calibrate gripper (homing). Usually done once after startup."""
        req = franka_pb2.GripperActionRequest(asynchronous=asynchronous)
        return self._call("HomingGripper", req).success

    # ------------------------------------------------------------------
    # Impedance Control / Kinesthetic Teaching
    # ------------------------------------------------------------------

    def set_joint_impedance(self, impedance: List[float]) -> bool:
        """Set joint impedance values (7 values, lower = more compliant)."""
        req = franka_pb2.JointValues(values=impedance)
        return self._call("SetJointImpedance", req).success

    def start_gravity_compensation(self, duration: float = 3600.0) -> bool:
        """Start gravity compensation mode for kinesthetic teaching.

        Args:
            duration: Duration in seconds (default: 1 hour).
        """
        req = franka_pb2.DurationRequest(duration=duration)
        return self._call("StartGravityCompensation", req).success

    def stop_motion(self) -> bool:
        """Stop any ongoing motion."""
        return self._call("StopMotion").success

    def close(self):
        """Close the gRPC channel."""
        if self.channel is not None:
            self.channel.close()


@click.command()
@click.option("--ip", default="192.168.1.7", help="Robot server IP address")
@click.option("--port", default=4242, help="Robot server port")
def main(ip, port):
    """Franka Robot Client CLI - Test connection and get robot state."""
    try:
        robot = FrankaRobotClient(f"{ip}:{port}")

        logger.info("Testing robot connection...")

        for _ in range(100):
            external_torques = robot.get_external_torques()
            logger.info(f"External Torques: {external_torques}")
            time.sleep(0.1)

        logger.info("Connection test successful!")
        robot.close()
    except Exception as e:
        logger.error(f"Connection test failed: {e}")


if __name__ == "__main__":
    main()
