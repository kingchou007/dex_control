"""High-level python wrapper for Franka robot control.

Author: Jinzhou Li

Provides a simplified interface for common robot operations:
- Moving end-effector by translation/rotation (delta or absolute)
- Moving joints to target positions
- Getting current robot state
- All control is non-blocking (optional) and asynchronous, so it's implemented in an event-driven manner.

Reference: https://timschneider42.github.io/franky/index.html
"""

import time
import click
from typing import List, Optional, Dict, Any, Union

from franky import (
    Affine,
    CartesianMotion,
    CartesianState,
    CartesianStopMotion,
    CartesianWaypoint,
    CartesianWaypointMotion,
    ElbowState,
    JointMotion,
    JointWaypoint,
    JointWaypointMotion,
    ReferenceType,
    RelativeDynamicsFactor,
    Robot,
    RobotPose,
    Twist,
)
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import cprint
import zerorpc


class FrankaWrapper:
    """Wrapper class for controlling the Franka robot."""

    def __init__(
        self,
        ip: str,
        controller_type: str = "joint_impedance",
        franka_hand: bool = False,
    ):
        """Initialize the FrankaWrapper.

        Args:
            ip: IP address of the robot.
            controller_type: Type of controller to use ("joint_impedance" or "cartesian_impedance").
            franka_hand: Whether the Franka hand is attached. Default is False, we use robotiq hand instead.
        """
        self.ip = ip
        self.robot = Robot(ip)
        self.robot.recover_from_errors()
        self.franka_hand = franka_hand
        self._setup_franka_hand()
        self._setup_controller(controller_type)


    def _setup_controller(self, controller_type: str):
        """Set up controller parameters based on the chosen controller type.

        Args:
            controller_type: Type of controller ("cartesian_impedance" or "joint_impedance").

        Raises:
            ValueError: If an invalid controller type is provided.
        """
        if controller_type == "cartesian_impedance":
            self.robot.set_cartesian_impedance([
                1000.0, 1000.0, 1000.0,
                30.0, 30.0, 30.0
            ])
        elif controller_type == "joint_impedance":
            self.robot.set_joint_impedance([100, 100, 100, 10, 10, 10, 10])
        else:
            raise ValueError(f"Invalid controller type: {controller_type}")

        # set relative dynamics factor to 0.1
        self.robot.relative_dynamics_factor = 0.1

        # TOD0: add more controller parameters can be customized -> config file
        cprint(f"[FrankaWrapper] Set up controller type: {controller_type}", "green")


    def _setup_franka_hand(self):
        """Set up franka hand parameters."""
        if self.franka_hand:
            try:
                from franky import Gripper
                self.gripper = Gripper(self.ip)
            except ImportError:
                cprint("[FrankaWrapper] Warning: Gripper not found in franky.", "yellow")
            
            self.speed = 0.02  # [m/s]
            self.force = 20.0  # [N]

        cprint("[FrankaWrapper] Set up franka hand", "green")


    def move_gripper(self, target_width: float, asynchronous: bool = False):
        """Move the gripper to the given target width.

        Args:
            target_width: Target width of the gripper [m].
            asynchronous: If True, the gripper move is asynchronous.
        """
        if not self.franka_hand:
             cprint("[FrankaWrapper] Franka hand not enabled.", "yellow")
             return

        if asynchronous:
            self.gripper.move_async(target_width, self.speed)
        else:
            self.gripper.move(target_width, self.speed)


    def grasp_object(self):
        """Grasp the object with unknown width."""
        if not self.franka_hand:
             cprint("[FrankaWrapper] Franka hand not enabled.", "yellow")
             return
        self.gripper.grasp(0.0, self.speed, self.force, epsilon_outer=1.0)


    def release_object(self):
        """Release the object."""
        if not self.franka_hand:
            cprint("[FrankaWrapper] Franka hand not enabled.", "yellow")
            return
        self.gripper.open(self.speed)


    def get_state(self):
        """Retrieve current robot state.

        Returns:
            dict: Dictionary containing all available robot state information. No fixed format—use as needed.
            
        See also:
            https://timschneider42.github.io/franky/structfranky_1_1_robot_state.html
        """

        state = self.robot.state  
        return {
            "q": np.array(state.q).tolist(),
            "q_d": np.array(state.q_d).tolist(),
            "dq": np.array(state.dq).tolist(),
            "dq_d": np.array(state.dq_d).tolist(),
            "ddq_d": np.array(state.ddq_d).tolist(),
            "tau_J": np.array(state.tau_J).tolist(),
            "tau_J_d": np.array(state.tau_J_d).tolist(),
            "dtau_J": np.array(state.dtau_J).tolist(),
            "O_T_EE": np.array(state.O_T_EE).tolist(),
        }


    def get_joint_positions(self):
        """Get the current joint positions of the robot.

        Returns:
            List[float]: Array of joint angles [q1, q2, ..., q7].
                        Converted to list for RPC serialization.
        """
        # Convert to list for RPC serialization
        return np.array(self.robot.state.q).tolist()


    def get_ee_pose(self) -> Dict[str, List[float]]:
        """Get the end-effector translation and quaternion.

        Returns:
            dict: Dictionary containing:
                "t": List of translation components [x, y, z].
                "q": List of quaternion components [x, y, z, w].
        """
        ee = self.robot.state.O_T_EE
        return {
            "t": np.array(ee.translation).tolist(),
            "q": np.array(ee.quaternion).tolist(),
        }


    def move_ee_pose(
        self,
        target_ee_pose: Any,
        asynchronous: bool = True,
        delta: bool = True
    ):
        """Move the end-effector to a target pose.

        Args:
            target_ee_pose: The target pose (format depends on franky.CartesianMotion).
            asynchronous: If True, return immediately without waiting for motion to complete.
            delta: If True, interpret target_ee_pose as relative to current pose.
                   If False, interpret as absolute pose.

        See also: 
            https://timschneider42.github.io/franky/classfranky_1_1_cartesian_motion.html
        """
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute

        pose_len = len(target_ee_pose)
        if pose_len == 3:
            # Only translation: [x, y, z]
            translation = list(target_ee_pose)
            target_ee_pose = Affine(translation)
        elif pose_len == 4:
            # Only rotation: [qx, qy, qz, qw], zero translation
            quaternion = list(target_ee_pose)
            target_ee_pose = Affine([0.0, 0.0, 0.0], quaternion)
        elif pose_len == 7:
            # Full pose: [x, y, z, qx, qy, qz, qw]
            translation = list(target_ee_pose[:3])
            quaternion = list(target_ee_pose[3:])
            target_ee_pose = Affine(translation, quaternion)
        else:
            raise ValueError(
                f"Invalid target_ee_pose length: {pose_len}. "
                "Expected 3, 4, or 7."
            )

        target_motion=CartesianMotion(target_ee_pose, reference_type=reference_type)

        self.robot.move(target_motion, asynchronous=asynchronous)


    def move_joints(
        self,
        target_joint_positions: List[float],
        asynchronous: bool = False
    ):
        """Move joints to the given target positions.

        Args:
            target_joint_positions: List of target joint angles.
            asynchronous: If True, return immediately without waiting for motion to complete.

        See also: 
            https://timschneider42.github.io/franky/classfranky_1_1_joint_motion.html
        """
        joint_motion = JointMotion(target_joint_positions)
        self.robot.move(joint_motion, asynchronous=asynchronous)

    # TODO: Test on real robot
    def move_ee_waypoints(self, waypoints: List[Any], delta: bool = True):
        """Move the end-effector through a sequence of waypoints.

        Args:
            waypoints: List of Cartesian poses/waypoints.
            delta: If True, interpret waypoints as relative motions.

        See also: 
            https://timschneider42.github.io/franky/classfranky_1_1_cartesian_waypoint_motion.html
        """
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute
        wp_motion = CartesianWaypointMotion([
            CartesianWaypoint(
                RobotPose(Affine(list(pt)), ElbowState(0.0)), 
                reference_type
            )
            for pt in waypoints
        ])
        self.robot.move(wp_motion)

    # TODO: Test on real robot
    def move_joint_waypoints(self, waypoints: List[List[float]]):
        """Move the robot's joints through a sequence of joint angle waypoints.

        Args:
            waypoints: List of joint configurations (each a list of angles).

        See also: 
            https://timschneider42.github.io/franky/classfranky_1_1_joint_waypoint_motion.html
        """
        wp_motion = JointWaypointMotion([
            JointWaypoint(list(pt))
            for pt in waypoints
        ])
        self.robot.move(wp_motion)


    def move_ee_cartesian_velocity(self):
        """Control EE using a specified Cartesian velocity (Not yet implemented)."""
        # TODO: implement this
        pass


    def move_joint_cartesian_velocity(self):
        """Control robot with joint or Cartesian velocity commands (Not yet implemented)."""
        # TODO: implement this
        pass

@click.command()
@click.option("--ip", default="192.168.1.33", help="Robot IP address")
@click.option("--controller-type", default="joint_impedance", help="Controller type")
@click.option("--port", default=4242, help="Port number")
def main(ip, controller_type, port):
    server = FrankaWrapper(ip, controller_type, franka_hand=False)
    s = zerorpc.Server(server, heartbeat=None)
    s.bind(f"tcp://0.0.0.0:{port}")
    cprint(f"Franka Server listening on tcp://0.0.0.0:{port}", "green")
    s.run()

if __name__ == "__main__":
    main()