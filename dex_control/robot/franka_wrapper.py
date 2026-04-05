"""High-level Python wrapper for Franka robot control.

Reference: https://timschneider42.github.io/franky/index.html
"""

import time
from typing import List, Dict, Any, Union

import numpy as np
import spdlog
from franky import (
    Robot, Gripper, Affine,
    CartesianMotion, JointMotion, JointVelocityMotion,
    ReferenceType, ControlException,
)
from omegaconf import DictConfig


class FrankaWrapper:
    """Wrapper class for controlling the Franka robot."""

    def __init__(self, cfg: DictConfig):
        self.cfg = cfg
        self.log = spdlog.ConsoleLogger("FrankaWrapper")
        self.log.set_level(spdlog.LogLevel.INFO)

        robot_ip = cfg.server.robot_ip
        self.log.info(f"Initializing FrankaWrapper with robot IP: {robot_ip}")
        self.robot_ip = robot_ip
        self.robot = Robot(robot_ip)
        self.teleop = cfg.get("teleop", False)
        self.robot.recover_from_errors()

        self.franka_hand = cfg.gripper.enabled
        self._gravity_comp_active = False
        self._setup_franka_hand()
        self._setup_home_position()
        self._setup_controller(cfg.controller.type)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _setup_home_position(self):
        home = list(self.cfg.home_position)
        self.robot.relative_dynamics_factor = 0.2
        self.robot.move(JointMotion(home), asynchronous=False)
        time.sleep(0.1)
        self.log.info("Moved to home position")

    def _setup_controller(self, controller_type: str):
        col = self.cfg.collision_thresholds
        self.robot.set_collision_behavior(
            list(col.joint), list(col.joint),
            list(col.cartesian), list(col.cartesian),
        )

        ctrl = self.cfg.controller
        if controller_type == "cartesian_impedance":
            self.robot.set_cartesian_impedance(list(ctrl.cartesian_impedance))
        elif controller_type == "joint_impedance":
            self.robot.set_joint_impedance(list(ctrl.joint_impedance))
        else:
            raise ValueError(f"Invalid controller type: {controller_type}")

        self.robot.relative_dynamics_factor = ctrl.dynamics_factor
        self.log.info(f"Controller: {controller_type}, dynamics_factor: {ctrl.dynamics_factor}")

    def _setup_franka_hand(self):
        if self.franka_hand:
            self.gripper = Gripper(self.robot_ip)
            self.speed = self.cfg.gripper.speed
            self.force = self.cfg.gripper.force
        self.log.info(f"Franka hand enabled: {self.franka_hand}")

    # ------------------------------------------------------------------
    # Error Recovery
    # ------------------------------------------------------------------

    def recover_from_errors(self) -> bool:
        try:
            self.robot.recover_from_errors()
            self.log.info("Recovered from errors")
            return True
        except Exception as e:
            self.log.error(f"Recovery failed: {e}")
            return False

    def robust_execute_motion(self, motion, asynchronous: bool = False, max_retries: int = 3) -> bool:
        """Execute motion with automatic retry on ControlException."""
        for attempt in range(max_retries):
            try:
                self.robot.move(motion, asynchronous=asynchronous)
                return True
            except ControlException as e:
                self.log.warn(f"Attempt {attempt+1}/{max_retries}: {e}")
                self.robot.recover_from_errors()

        self.log.error(f"Failed after {max_retries} attempts")
        return False

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_state(self) -> Dict[str, List[float]]:
        """Get full robot state."""
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
        return np.array(self.robot.state.q).tolist()

    def get_ee_pose(self) -> Dict[str, List[float]]:
        """Returns dict with 't' [x,y,z] and 'q' [qx,qy,qz,qw]."""
        ee = self.robot.state.O_T_EE
        return {
            "t": np.array(ee.translation).tolist(),
            "q": np.array(ee.quaternion).tolist(),
        }

    def get_torques(self) -> List[float]:
        return np.array(self.robot.state.tau_J).tolist()

    def get_external_torques(self) -> List[float]:
        return np.array(self.robot.state.tau_ext_hat_filtered).tolist()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_ee_pose(self, target_ee_pose: Union[List[float], Affine],
                     asynchronous: bool = True, delta: bool = True) -> bool:
        """Move EE to target pose.

        Args:
            target_ee_pose: [x,y,z] | [qx,qy,qz,qw] | [x,y,z,qx,qy,qz,qw] or Affine.
            asynchronous: Return immediately without waiting.
            delta: Interpret as relative to current pose.
        """
        reference_type = ReferenceType.Relative if delta else ReferenceType.Absolute

        if isinstance(target_ee_pose, list):
            n = len(target_ee_pose)
            if n == 3:
                target_ee_pose = Affine(target_ee_pose)
            elif n == 4:
                target_ee_pose = Affine([0.0, 0.0, 0.0], target_ee_pose)
            elif n == 7:
                target_ee_pose = Affine(target_ee_pose[:3], target_ee_pose[3:])
            else:
                raise ValueError(f"Invalid pose length: {n}. Expected 3, 4, or 7.")

        motion = CartesianMotion(target_ee_pose, reference_type=reference_type)
        return self.robust_execute_motion(motion, asynchronous=asynchronous)

    def move_joints(self, target_joint_positions: List[float],
                    asynchronous: bool = False) -> bool:
        return self.robust_execute_motion(
            JointMotion(target_joint_positions), asynchronous=asynchronous
        )

    def set_joint_impedance(self, impedance: List[float]) -> bool:
        self.robot.set_joint_impedance(impedance)
        self.log.info(f"Joint impedance: {impedance}")
        return True

    def start_gravity_compensation(self, duration: float = 3600.0,
                                   joint_impedance: float = 0.0) -> bool:
        """Start gravity compensation mode for kinesthetic teaching.

        Args:
            duration: Duration in seconds.
            joint_impedance: Impedance per joint (0=fully compliant, 50=stiff).
        """
        imp = [joint_impedance] * 7
        self.robot.set_joint_impedance(imp)
        self.log.info(f"Gravity compensation: impedance={joint_impedance}, duration={duration}s")

        try:
            zero_vel = [0.0] * 7
            self.robot.move(JointVelocityMotion(zero_vel, duration), asynchronous=True)
            self._gravity_comp_active = True
            return True
        except ControlException as e:
            self.log.error(f"Failed to start gravity compensation: {e}")
            self._gravity_comp_active = False
            return False

    def is_gravity_compensation_active(self) -> bool:
        return self._gravity_comp_active

    def stop_gravity_compensation(self) -> bool:
        self._gravity_comp_active = False
        return self.stop_motion()

    def stop_motion(self) -> bool:
        try:
            self.robot.stop()
            return True
        except Exception as e:
            self.log.error(f"Failed to stop motion: {e}")
            return False

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    def move_gripper(self, target_width: float, speed: float = None,
                     asynchronous: bool = False) -> bool:
        if not self.franka_hand:
            return False
        s = speed or self.speed
        if asynchronous:
            self.gripper.move_async(target_width, s)
            return True
        return self.gripper.move(target_width, s)

    def grasp_object(self, width: float = 0.0, speed: float = None,
                     force: float = None, epsilon_inner: float = 0.005,
                     epsilon_outer: float = 0.005, asynchronous: bool = False) -> bool:
        if not self.franka_hand:
            return False
        s = speed or self.speed
        f = force or self.force
        if asynchronous:
            self.gripper.grasp_async(width, s, f, epsilon_inner, epsilon_outer)
            return True
        return self.gripper.grasp(width, s, f, epsilon_inner, epsilon_outer)

    def open_gripper(self, speed: float = None, asynchronous: bool = False) -> bool:
        if not self.franka_hand:
            return False
        s = speed or self.speed
        if asynchronous:
            self.gripper.open_async(s)
        else:
            self.gripper.open(s)
        return True

    def stop_gripper(self, asynchronous: bool = False) -> bool:
        if not self.franka_hand:
            return False
        if asynchronous:
            self.gripper.stop_async()
        else:
            self.gripper.stop()
        return True

    def homing_gripper(self, asynchronous: bool = False) -> bool:
        if not self.franka_hand:
            return False
        if asynchronous:
            self.gripper.homing_async()
        else:
            self.gripper.homing()
        return True

    def get_gripper_width(self) -> float:
        if not self.franka_hand:
            return 0.0
        return self.gripper.read_once().width

    def get_gripper_max_width(self) -> float:
        if not self.franka_hand:
            return 0.0
        return self.gripper.read_once().max_width

    def is_gripper_grasped(self) -> bool:
        if not self.franka_hand:
            return False
        return self.gripper.read_once().is_grasped
