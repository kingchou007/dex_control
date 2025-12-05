import time
import zmq
import numpy as np
from collections.abc import Callable

from dex_control.teleop.controller import Controller
from dex_control.utils.misc.misc_utils import run_threaded_command


class GELLODevice:
    def __init__(self):
        super().__init__()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://127.0.0.1:5555")
        self.callbacks = {}

    def add_callback(self, key: str, func: Callable):
        self.callbacks[key] = func

    def advance(self):
        self.socket.send(b"get_joint_state")
        message = self.socket.recv()
        gello_action = np.frombuffer(message, dtype=np.float32)
        return gello_action


class Gello(Controller):
    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 3,
        rmat_reorder: list = [-2, -1, -3, 4],
    ):
        self.action_space = "joint_position"
        self.gripper_action_space = "velocity"
        self.gello_device = GELLODevice()
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        self.reset_state()

        self.running = True
        run_threaded_command(self._update_internal_state)

        print("Warning: GELLO controller is experimental!")
        print("Since the GELLO motors are not strong enough to hold itself up, it cannot easily match the Franka. During init, the Franka will jerk toward the GELLO's joint positions.")
        print("This should be replaced by FACTR once it's released.")

    def reset_state(self):
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        last_read_time = time.time()
        while self.running:
            time.sleep(1 / hz)
            time_since_read = time.time() - last_read_time
            
            gello_state = self.gello_device.advance()
            gello_joints = gello_state[:-1]
            gello_gripper = gello_state[-1]
            movement_enabled = True

            self._state["controller_on"] = time_since_read < num_wait_sec

            toggled = self._state["movement_enabled"] != movement_enabled
            self.update_sensor = self.update_sensor or movement_enabled

            # Save Info #
            # TODO: Save GELLO info here instead
            self._state["joints"] = gello_joints
            self._state["gripper"] = gello_gripper


            self._state["movement_enabled"] = movement_enabled
            self._state["controller_on"] = True
            last_read_time = time.time()

    def _process_reading(self):
        self.gello_state = {"joints": self._state["joints"], "gripper": self._state["gripper"]}

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, state_dict):
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        info_dict = {"target_joint_positions": self.gello_state["joints"], "target_gripper_position": self.gello_state["gripper"]}
        action = np.concatenate([self.gello_state["joints"], [self.gello_state["gripper"]]])

        return action, info_dict

    def get_info(self):
        return {
            "success": self._state["buttons"]["A"],
            "failure": self._state["buttons"]["B"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }

    def forward(self, obs_dict):
        return self._calculate_action(obs_dict["robot_state"])
    
    def register_key(self, key):
        pass
    
    def close(self):
        self.running = False