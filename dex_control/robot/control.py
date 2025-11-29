# This file is part of the dex_control package.
# Author: Jinzhou Li
# Use of this source code is governed by the MIT license, see LICENSE
# We acknowledge Franka Robotics GmbH example code and EVA for the inspiration.

import zerorpc
import numpy as np
from pylibfranka import Robot, ControllerMode, JointPositions
import time

class FrankaServer:
    def __init__(self, ip: str):
        """Simple Franka state server running on the NUC.

        Args:
            ip (str): Franka robot IP (FCI).
        """
        self.robot = Robot(ip)

        # TODO: decide if you want to override the default collision behavior.
        # These thresholds are copied from Franka example; you can remove this
        # block if you prefer using the robot's current configuration.
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        self.robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

    def get_state(self):
        """Read one robot state and return a dictionary of the state.

        Returns:
            dict:
                - "joint_positions": Joint positions (7D)
                - "joint_velocities": Joint velocities (7D)
                - "joint_torques": Joint torques (7D)
                - "external_joint_torques": Filtered external joint torques (7D)
                - "end_effector_position": End-effector position [x, y, z] (3D)
                - "end_effector_orientation": End-effector orientation as 3x3
                  rotation matrix, flattened to 9D (row-major)
                - "external_wrench": External wrench in base frame
                  [fx, fy, fz, mx, my, mz] (6D)
        """
        state_raw = self.robot.read_once()

        # O_T_EE: 4x4 transform flattened to 16 elements.
        # Layout: [R00, R01, R02, Tx,
        #          R10, R11, R12, Ty,
        #          R20, R21, R22, Tz,
        #          0,   0,   0,   1 ]
        O_T_EE = np.array(state_raw.O_T_EE, dtype=float)

        # Translation (3D)
        ee_position = O_T_EE[12:15].tolist()

        # Rotation matrix (row-major, 3x3 → 9D)
        # Indices of R:
        #   row 0: 0, 1, 2
        #   row 1: 4, 5, 6
        #   row 2: 8, 9, 10
        ee_orientation = O_T_EE[[0, 1, 2, 4, 5, 6, 8, 9, 10]].tolist()

        state = {
            "joint_positions": np.array(state_raw.q, dtype=float).tolist(),
            "joint_velocities": np.array(state_raw.dq, dtype=float).tolist(),
            "joint_torques": np.array(state_raw.tau_J, dtype=float).tolist(),
            "end_effector_position": ee_position,
            "end_effector_orientation": ee_orientation,
            "external_wrench": np.array(state_raw.O_F_ext_hat_K, dtype=float).tolist(),
            "external_joint_torques": np.array(
                state_raw.tau_ext_hat_filtered, dtype=float
            ).tolist(),
        }

        return state


    def reset_to_joints(self, target_joint_positions: list[float], duration: float = 5.0):
        """Smoothly move robot from current to target joint positions (blocking).

        Args:
            target_joint_positions: 7D absolute joint angles [rad]
            duration: Motion duration in seconds
        """
        if len(target_joint_positions) != 7:
            raise ValueError(f"Expected 7 joint values, got {len(target_joint_positions)}")

        initial_pos = np.array(self.get_state()['joint_positions'], dtype=float)
        target_pos = np.array(target_joint_positions, dtype=float)
        
        control = self.robot.start_joint_position_control(ControllerMode.CartesianImpedance)
        elapsed = 0.0

        while True:
            _, dt = control.readOnce()
            elapsed += dt.to_sec()
            
            # Smooth cosine interpolation
            progress = min(elapsed / duration, 1.0)
            alpha = 0.5 * (1.0 - np.cos(np.pi * progress))

            # Interpolate between initial and target positions (avoid sudden jumps)
            current_pos = initial_pos + alpha * (target_pos - initial_pos)
            
            cmd = JointPositions(current_pos.tolist())
            
            # Check if the motion has finished
            if elapsed >= duration:
                cmd.motion_finished = True
                control.writeOnce(cmd)
                break
            control.writeOnce(cmd)


if __name__ == "__main__":
    ip = "192.168.1.33"
    server = FrankaServer(ip)
    s = zerorpc.Server(server, heartbeat=None)
    s.bind("tcp://0.0.0.0:4242")
    print("FrankaServer listening on tcp://0.0.0.0:4242")
    s.run()

