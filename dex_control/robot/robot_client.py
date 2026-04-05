"""Client for Franka Robot Control Server.

Usage:
    from dex_control.robot.robot_client import FrankaRobotClient

    robot = FrankaRobotClient(server_addr="tcp://192.168.1.7:4242")
    pose = robot.get_ee_pose()
    robot.move_ee_pose([0.0, 0.0, 0.01], delta=True)
"""

import time
from typing import List, Dict, Any

import numpy as np
import zerorpc
import spdlog


class FrankaRobotClient:
    """Robot client for controlling the Franka robot remotely via zerorpc."""

    def __init__(self, server_addr: str = "tcp://192.168.1.7:4242", max_retries: int = 3):
        self.server_addr = server_addr
        self.max_retries = max_retries
        self.log = spdlog.ConsoleLogger("FrankaClient")
        self.log.set_level(spdlog.LogLevel.INFO)
        self.client = None
        self._connect()

    def _connect(self):
        self.client = zerorpc.Client(heartbeat=20, timeout=60)
        self.client.connect(self.server_addr)
        self.log.info(f"Connected to {self.server_addr}")

    def _reconnect(self):
        """Close old client and reconnect."""
        self.log.info("Reconnecting...")
        try:
            if self.client:
                self.client.close()
        except Exception:
            pass
        time.sleep(0.5)
        self._connect()

    def _call(self, method: str, *args):
        """Call remote method with automatic retry and reconnection.

        Retries on connection/timeout errors. Raises immediately on
        RemoteError (server-side logic errors like bad arguments).
        """
        last_err = None
        for attempt in range(1, self.max_retries + 1):
            try:
                return getattr(self.client, method)(*args)
            except zerorpc.exceptions.RemoteError:
                raise
            except (zerorpc.exceptions.TimeoutExpired,
                    zerorpc.exceptions.LostRemote,
                    ConnectionError, OSError) as e:
                last_err = e
                self.log.warn(f"[{method}] attempt {attempt}/{self.max_retries}: {e}")
                if attempt < self.max_retries:
                    self._reconnect()
            except Exception as e:
                last_err = e
                self.log.error(f"[{method}] attempt {attempt}/{self.max_retries}: {e}")
                if attempt < self.max_retries:
                    self._reconnect()

        self.log.error(f"[{method}] all {self.max_retries} attempts failed")
        raise last_err

    def close(self):
        if self.client:
            self.client.close()
            self.client = None

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_joint_positions(self) -> np.ndarray:
        return np.array(self._call("get_joint_positions"))

    def get_ee_pose(self) -> Dict[str, np.ndarray]:
        """Returns dict with 't' [x,y,z] and 'q' [qx,qy,qz,qw]."""
        pose = self._call("get_ee_pose")
        return {"t": np.array(pose["t"]), "q": np.array(pose["q"])}

    def get_external_torques(self) -> np.ndarray:
        return np.array(self._call("get_external_torques"))

    def get_torques(self) -> np.ndarray:
        return np.array(self._call("get_torques"))

    def get_state(self) -> Dict[str, Any]:
        return self._call("get_state")

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_ee_pose(self, target_ee_pose: List[float],
                     asynchronous: bool = True, delta: bool = True) -> bool:
        """Move EE. target: [x,y,z] | [qx,qy,qz,qw] | [x,y,z,qx,qy,qz,qw]."""
        return self._call("move_ee_pose", target_ee_pose, asynchronous, delta)

    def move_joints(self, target_joint_positions: List[float],
                    asynchronous: bool = False) -> bool:
        return self._call("move_joints", target_joint_positions, asynchronous)

    def set_joint_impedance(self, impedance: List[float]) -> bool:
        return self._call("set_joint_impedance", impedance)

    def start_gravity_compensation(self, duration: float = 3600.0,
                                   joint_impedance: float = 0.0) -> bool:
        return self._call("start_gravity_compensation", duration, joint_impedance)

    def is_gravity_compensation_active(self) -> bool:
        return self._call("is_gravity_compensation_active")

    def stop_gravity_compensation(self) -> bool:
        return self._call("stop_gravity_compensation")

    def stop_motion(self) -> bool:
        return self._call("stop_motion")

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    def move_gripper(self, target_width: float, speed: float = 0.1,
                     asynchronous: bool = False) -> bool:
        return self._call("move_gripper", target_width, speed, asynchronous)

    def open_gripper(self, speed: float = 0.1, asynchronous: bool = False) -> bool:
        return self._call("open_gripper", speed, asynchronous)

    def release_object(self) -> bool:
        return self.open_gripper()

    def grasp_object(self, width: float = 0.0, speed: float = 0.1,
                     force: float = 60.0, epsilon_inner: float = 0.005,
                     epsilon_outer: float = 0.005, asynchronous: bool = False) -> bool:
        return self._call("grasp_object", width, speed, force,
                          epsilon_inner, epsilon_outer, asynchronous)

    def stop_gripper(self, asynchronous: bool = False) -> bool:
        return self._call("stop_gripper", asynchronous)

    def homing_gripper(self, asynchronous: bool = False) -> bool:
        return self._call("homing_gripper", asynchronous)

    def get_gripper_width(self) -> float:
        return float(self._call("get_gripper_width"))

    def get_gripper_max_width(self) -> float:
        return float(self._call("get_gripper_max_width"))

    def is_gripper_grasped(self) -> bool:
        return bool(self._call("is_gripper_grasped"))
