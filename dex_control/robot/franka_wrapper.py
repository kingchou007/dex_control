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
from typing import List, Optional, Dict, Any, Union

import click
import numpy as np
import spdlog
import zerorpc
from franky import *
from scipy.spatial.transform import Rotation as R


class FrankaWrapper:
    """Wrapper class for controlling the Franka robot."""

    def __init__(
        self,
        ip: str,
        controller_type: str = "joint_impedance",
        franka_hand: bool = False,
        teleop: bool = False,
    ):
        """Initialize the FrankaWrapper.

        Args:
            ip: IP address of the robot.
            controller_type: Type of controller to use ("joint_impedance" or "cartesian_impedance").
            franka_hand: Whether the Franka hand is attached. Default is False, we use robotiq hand instead.
            teleop: Whether this is for teleoperation (affects dynamics settings).
        """
        # Initialize logger
        self.log = spdlog.ConsoleLogger("FrankaWrapper")
        self.log.set_level(spdlog.LogLevel.INFO)

        self.log.info(f"Initializing FrankaWrapper with IP: {ip}")
        self.ip = ip
        self.robot = Robot(ip)
        self.teleop = teleop
        self.robot.recover_from_errors()
        self.franka_hand = franka_hand
        self._setup_franka_hand()
        self._setup_home_position()
        self._setup_controller_parameters(controller_type)

    def _setup_home_position(self):
        """Set up home position (faster)."""
        joint_motion = JointMotion([0.09162, -0.19826, -0.01990, -2.47323, -0.01307, 2.30397, 0.84809])
        self.robot.relative_dynamics_factor = 0.2
        self.robot.move(joint_motion, asynchronous=False)
        time.sleep(0.1)
        self.log.info("Set up home position (fast)")

    def _setup_controller_parameters(self, controller_type: str):
        """Set up controller parameters based on the chosen controller type.

        Args:
            controller_type: Type of controller ("cartesian_impedance" or "joint_impedance").

        Raises:
            ValueError: If an invalid controller type is provided.

        Notes:
            - Sets the robot's collision behavior thresholds (currently all set to 100).
            - For "cartesian_impedance", sets cartesian impedance to [1000, 1000, 1000, 30, 30, 30].
            - For "joint_impedance", sets soft joint impedance ([50.0, ...]).
                * The stiffness is only available during the robot moving, not when stopped.
                * Might require a long-term running loop to keep the stiffness active.
                * Example settings for stiffer impedance:
                    self.robot.set_joint_impedance([300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0])
                    self.robot.set_joint_impedance([800.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0])
            - The relative dynamics factor is set to 0.1.
                * Lower = slower but smoother; Higher = faster but more prone to errors.
                * For safe teleoperation, a value of 0.05 is typical; tune as needed for your application.
        """
        self.log.info("Set collision behavior")
        self.robot.set_collision_behavior(
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
        )

        self.log.info(f"Set Controller Type: {controller_type}")
        if controller_type == "cartesian_impedance":
            self.robot.set_cartesian_impedance([
                1000.0, 1000.0, 1000.0,
                30.0, 30.0, 30.0
            ])
        elif controller_type == "joint_impedance":
            self.robot.set_joint_impedance([50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0])
        else:
            raise ValueError(f"Invalid controller type: {controller_type}")

        self.robot.relative_dynamics_factor = 0.1

    def _setup_franka_hand(self):
        """Set up franka hand parameters."""
        if self.franka_hand:
            try:
                from franky import Gripper
                self.gripper = Gripper(self.ip)
            except ImportError:
                self.log.warn("Warning: Gripper not found in franky.")

            self.speed = 0.02  # [m/s]
            self.force = 20.0  # [N]

        self.log.info("Set up franka hand")

    def recover_from_errors(self):
        """Recover from errors automatically.

        Call this method when the robot enters reflex/error state to attempt
        automatic recovery without manual intervention.

        Returns:
            bool: True if recovery succeeded, False otherwise.
        """
        try:
            self.robot.recover_from_errors()
            self.log.info("Successfully recovered from errors")
            return True
        except Exception as e:
            self.log.error(f"Failed to recover from errors: {e}")
            return False

    def robust_execute_motion(self, motion, asynchronous: bool = False, max_retries: int = 3):
        """Execute a motion command with automatic retry on ControlException.

        Handles common errors like:
        - Reflex errors (collision detection triggered)
        - Communication errors or disconnections
        - Motion aborted due to safety limits

        The method automatically attempts error recovery between retries.

        Args:
            motion: The motion object (CartesianMotion, JointMotion, etc.)
            asynchronous: If True, return immediately without waiting for motion to complete.
            max_retries: Maximum number of retry attempts (default: 3).

        Returns:
            bool: True if motion succeeded, False if all retries failed.

        See also:
            https://timschneider42.github.io/franky/classfranky_1_1_control_exception.html
        """
        for attempt in range(max_retries):
            try:
                self.robot.move(motion, asynchronous=asynchronous)
                return True
            except ControlException as e:
                # Common ControlException causes: reflex (collision), disconnection, safety violations
                if attempt < max_retries - 1:
                    self.log.warn(f"Attempt {attempt+1}/{max_retries}: ControlException (reflex/error): {e}")
                    self.log.info("Attempting automatic recovery...")
                else:
                    self.log.error(f"Attempt {attempt+1}/{max_retries}: ControlException: {e}")
                self.robot.recover_from_errors()

        # If we get here, all attempts failed
        self.log.error(f"Failed to move after {max_retries} attempts. Manual intervention may be required.")
        return False

    def move_gripper(self, target_width: float, asynchronous: bool = False):
        """Move the gripper to the given target width.

        Args:
            target_width: Target width of the gripper [m].
            asynchronous: If True, the gripper move is asynchronous.
        """
        if not self.franka_hand:
            self.log.warn("Franka hand not enabled.")
            return

        if asynchronous:
            self.gripper.move_async(target_width, self.speed)
        else:
            self.gripper.move(target_width, self.speed)

    def grasp_object(self):
        """Grasp the object with unknown width."""
        if not self.franka_hand:
            self.log.warn("Franka hand not enabled.")
            return
        self.gripper.grasp(0.0, self.speed, self.force, epsilon_outer=1.0)

    def release_object(self):
        """Release the object."""
        if not self.franka_hand:
            self.log.warn("Franka hand not enabled.")
            return
        self.gripper.open(self.speed)

    def get_state(self) -> Dict[str, List[float]]:
        """Retrieve current robot state.

        Returns:
            dict: Dictionary containing all available robot state information.

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

    def get_joint_positions(self) -> List[float]:
        """Get the current joint positions of the robot.

        Returns:
            List[float]: List of joint angles [q1, q2, ..., q7].
                        Converted to list for RPC serialization.
        """
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
        target_ee_pose: Union[List[float], Affine],
        asynchronous: bool = True,
        delta: bool = True
    ) -> bool:
        """Move the end-effector to a target pose.

        Args:
            target_ee_pose: Target pose as [x, y, z], [qx, qy, qz, qw], or [x, y, z, qx, qy, qz, qw].
            asynchronous: If True, return immediately without waiting for motion to complete.
            delta: If True, interpret target_ee_pose as relative to current pose.
                   If False, interpret as absolute pose.

        Returns:
            bool: True if motion succeeded, False if all retries failed.

        See also:
            https://timschneider42.github.io/franky/classfranky_1_1_cartesian_motion.html
        """
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute

        if isinstance(target_ee_pose, list):
            pose_len = len(target_ee_pose)
            if pose_len == 3:
                target_ee_pose = Affine(target_ee_pose)
            elif pose_len == 4:
                target_ee_pose = Affine([0.0, 0.0, 0.0], target_ee_pose)
            elif pose_len == 7:
                target_ee_pose = Affine(target_ee_pose[:3], target_ee_pose[3:])
            else:
                raise ValueError(
                    f"Invalid target_ee_pose length: {pose_len}. "
                    "Expected 3, 4, or 7."
                )

        target_motion = CartesianMotion(target_ee_pose, reference_type=reference_type)
        return self.robust_execute_motion(target_motion, asynchronous=asynchronous)

    def move_joints(
        self,
        target_joint_positions: List[float],
        asynchronous: bool = False
    ) -> bool:
        """Move joints to the given target positions.

        Args:
            target_joint_positions: List of target joint angles.
            asynchronous: If True, return immediately without waiting for motion to complete.

        Returns:
            bool: True if motion succeeded, False if all retries failed.

        See also:
            https://timschneider42.github.io/franky/classfranky_1_1_joint_motion.html
        """
        joint_motion = JointMotion(target_joint_positions)
        return self.robust_execute_motion(joint_motion, asynchronous=asynchronous)

    def move_ee_waypoints(self, waypoints: List[Any], delta: bool = True):
        """Move the end-effector through a sequence of waypoints.

        Args:
            waypoints: List of Cartesian poses/waypoints.
            delta: If True, interpret waypoints as relative motions.

        See also:
            https://timschneider42.github.io/franky/classfranky_1_1_cartesian_waypoint_motion.html
        """
        raise NotImplementedError("Not tested on real robot")

    def move_joint_waypoints(self, waypoints: List[List[float]]):
        """Move the robot's joints through a sequence of joint angle waypoints.

        Args:
            waypoints: List of joint configurations (each a list of angles).

        See also:
            https://timschneider42.github.io/franky/classfranky_1_1_joint_waypoint_motion.html
        """
        raise NotImplementedError("Not tested on real robot")

    def move_ee_cartesian_velocity(self):
        """Control EE using a specified Cartesian velocity (Not yet implemented)."""
        raise NotImplementedError("Not yet implemented")

    def move_joint_cartesian_velocity(self):
        """Control robot with joint or Cartesian velocity commands (Not yet implemented)."""
        raise NotImplementedError("Not yet implemented")


@click.command()
@click.option("--ip", default="192.168.1.33", help="Robot IP address")
@click.option("--controller-type", default="joint_impedance", help="Controller type")
@click.option("--port", default=4242, help="Port number")
@click.option("--teleop", is_flag=True, default=True, help="Enable teleoperation")
def main(ip, controller_type, port, teleop):
    """Start Franka RPC server."""
    server = FrankaWrapper(ip, controller_type, franka_hand=False, teleop=teleop)
    s = zerorpc.Server(server, heartbeat=None)
    s.bind(f"tcp://0.0.0.0:{port}")
    server.log.info(f"Franka Server listening on tcp://0.0.0.0:{port}")
    server.log.info(f"Teleoperation: {teleop}")
    s.run()


if __name__ == "__main__":
    main()
