"""Client for Franka Robot Control Server.

Author: Jinzhou Li

This client is used to control the Franka robot using the Franka Robot Control Server.
It is implemented using zerorpc, a lightweight RPC framework for Python.

Please also refer to dex_control/robot/franka_wrapper.py for the server side implementation.
Before using this client, please make sure the Franka Robot Control Server is running.

See examples/ for usage examples.
"""

import time
from typing import List, Dict, Any, Union
from termcolor import cprint
import numpy as np
import zerorpc
import click


class FrankaRobotClient:
    """Robot Client for controlling the Franka robot remotely."""

    def __init__(self, server_addr: str = "tcp://192.168.1.7:4242"):
        """Initialize the FrankaRobotClient.

        Args:
            server_addr: Address of the RPC server (e.g., "tcp://192.168.1.7:4242").
        """
        self.client = zerorpc.Client()
        self.client.connect(server_addr)
        print(f"Connected to Franka RPC server at {server_addr}")

    def get_joint_positions(self) -> List[float]:
        """Get the current joint positions of the robot.

        Returns:
            List of joint angles.
        """
        return self.client.get_joint_positions()

    def get_ee_pose(self) -> Dict[str, List[float]]:
        """Get the current end-effector pose.

        Returns:
            Dictionary containing 't' (translation) and 'q' (quaternion).
        """
        return self.client.get_ee_pose()

    def get_state(self) -> Any:
        """Get the full robot state.

        Returns:
            The robot state object (serialized).
        """
        return self.client.get_state()

    def move_ee_pose(
        self,
        target_ee_pose: List[float],
        asynchronous: bool = True,
        delta: bool = True
    ) -> None:
        """Move EE to target pose.

        Args:
            target_ee_pose: Target pose as [x, y, z], [qx, qy, qz, qw],
                            or [x, y, z, qx, qy, qz, qw].
            asynchronous: If True, return immediately.
            delta: If True, interpret as relative motion.
        """
        return self.client.move_ee_pose(target_ee_pose, asynchronous, delta)

    def move_ee_waypoints(self, waypoints: List[List[float]], delta: bool = True) -> None:
        """Move EE through a sequence of waypoints.

        Args:
            waypoints: List of waypoints (poses).
            delta: If True, interpret waypoints as relative.
        """
        return self.client.move_ee_waypoints(waypoints, delta)

    def move_joint_waypoints(self, waypoints: List[List[float]]) -> None:
        """Move joints through a sequence of waypoints.

        Args:
            waypoints: List of joint configurations.
        """
        return self.client.move_joint_waypoints(waypoints)

    def move_joints(
        self,
        target_joint_positions: List[float],
        asynchronous: bool = False
    ) -> None:
        """Move joints to target positions.

        Args:
            target_joint_positions: List of target joint angles.
            asynchronous: If True, return immediately.
        """
        return self.client.move_joints(target_joint_positions, asynchronous)

    def move_gripper(self, target_width: float, asynchronous: bool = False) -> None:
        """Move gripper to target width.

        Args:
            target_width: Target width in meters.
            asynchronous: If True, return immediately.
        """
        return self.client.move_gripper(target_width, asynchronous)

    def grasp_object(self) -> None:
        """Grasp object with unknown width."""
        return self.client.grasp_object()

    def release_object(self) -> None:
        """Release object."""
        return self.client.release_object()


@click.command()
@click.option("--ip", default="192.168.1.7", help="Robot NUC server IP address")
def main(ip):
    """Franka Robot Client CLI"""
    robot = FrankaRobotClient(f"tcp://{ip}:4242")
    
if __name__ == "__main__":
    main()
