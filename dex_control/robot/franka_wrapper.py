"""
High-level python wrapper for Franka robot control.

Author: Jinzhou Li

Provides a simplified interface for common robot operations:
- Moving end-effector by translation/rotation (delta or absolute)
- Moving joints to target positions 
- Getting current robot state 
- All control is non-blocking (optional) and asynchronous, so it's implemented in an event-driven manner.

Reference: https://timschneider42.github.io/franky/index.html

"""

import time
import numpy as np
import zerorpc
from termcolor import cprint
from typing import Optional
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
from franky import (
    Affine,
    JointWaypointMotion,
    JointWaypoint,
    Robot,
    CartesianWaypointMotion,
    CartesianWaypoint,
    ReferenceType,
    RobotPose,
    ElbowState,
    CartesianMotion,
    CartesianState,
    Twist,
    RelativeDynamicsFactor,
    CartesianStopMotion,
)


class FrankaWrapper:
    def __init__(self, ip: str, controller_type: str = "joint_impedance"):
        self.ip = ip
        self.robot = Robot(ip)
        self._setup_controller(controller_type)
        cprint(f"FrankaWrapper initialized with controller type: {controller_type}", "green")

    def _setup_controller(self, controller_type: str):
        """Set up controller parameters based on the chosen controller type."""
        if controller_type == "cartesian_impedance":
            self.robot.set_cartesian_impedance([
                1000.0, 1000.0, 1000.0,   # x, y, z stiffness
                30.0, 30.0, 30.0          # roll, pitch, yaw stiffness
            ])
        elif controller_type == "joint_impedance":
            self.robot.set_joint_impedance([100, 100, 100, 10, 10, 10, 10])
        else:
            raise ValueError(f"Invalid controller type: {controller_type}")

    def get_state(self):
        """Retrieve current robot state and print key info, then return the state."""
        state = self.robot.state
        print("O_T_EE: ", state.O_T_EE)
        print("Joints: ", state.q)
        time.sleep(0.05)
        return state

    def get_joint_positions(self) -> np.ndarray:
        """
        Get the current joint positions of the robot.
        Returns:
            np.ndarray: Array of joint angles [q1, q2, ..., q7].
        """
        return self.robot.state.q

    def get_ee_pose(self):
        """
        Returns the end-effector pose as given by the robot API.
        Returns:
            pose: e.g., O_T_EE matrix or representation from the state.
        """
        return self.robot.state.O_T_EE

    def move_ee_pose(self, target_ee_pose, asynchronous: bool = False, delta: bool = True):
        """Move the end-effector to a target pose, as a delta or absolute pose."""
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute
        cart_motion = CartesianMotion(target_ee_pose, reference_type=reference_type)
        self.robot.move(cart_motion, asynchronous=asynchronous)

    def move_joints(self, target_joint_positions, asynchronous: bool = False):
        """Move joints to the given target positions."""
        joint_motion = JointMotion(target_joint_positions)
        self.robot.move(joint_motion, asynchronous=asynchronous)

    def move_ee_waypoints(self, waypoints, delta: bool = True):
        """Move the end-effector through a sequence of waypoints.
        Args:
            waypoints: List of 3D positions.
            delta (bool): If True, interpret waypoints as relative motions.
        """
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute
        wp_motion = CartesianWaypointMotion([
            CartesianWaypoint(RobotPose(Affine(list(pt)), ElbowState(0.0)), reference_type)
            for pt in waypoints
        ])
        self.robot.move(wp_motion)

    def move_joint_waypoints(self, waypoints):
        """Move the robot's joints through a sequence of joint angle waypoints."""
        wp_motion = JointWaypointMotion([
            JointWaypoint(list(pt))
            for pt in waypoints
        ])
        self.robot.move(wp_motion)

    def move_ee_cartesian_velocity(self):
        """Control EE using a specified Cartesian velocity (Not yet implemented)."""
        pass  # TODO: implement this

    def move_joint_cartesian_velocity(self):
        """Control robot with joint or Cartesian velocity commands (Not yet implemented)."""
        pass  # TODO: implement this


if __name__ == "__main__":
    # TODO: parse arguments as needed
    ip = "192.168.1.33"
    controller_type = "osc"
    port = 4242
    server = FrankaWrapper(ip, controller_type)
    s = zerorpc.Server(server, heartbeat=None)
    s.bind(f"tcp://0.0.0.0:{port}")
    cprint(f"Franka Server listening on tcp://0.0.0.0:{port}", "green")
    s.run()
