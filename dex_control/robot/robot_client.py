"""Client for Franka Robot Control Server.

Usage:
    from dex_control.robot.robot_client import FrankaRobotClient

    robot = FrankaRobotClient(server_addr="tcp://192.168.1.7:4242")
    pose = robot.get_ee_pose()
    robot.move_ee_pose([0.0, 0.0, 0.01], delta=True)
"""

import time
import functools
from typing import List, Dict, Any

import numpy as np
import zerorpc
import click
import spdlog


def robust_call(max_retries: int = 3, reconnect: bool = True):
    """Decorator to retry and reconnect failed zerorpc calls."""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            last_err = None
            for i in range(max_retries):
                try:
                    return func(self, *args, **kwargs)
                except (zerorpc.exceptions.RemoteError,
                        zerorpc.exceptions.TimeoutExpired) as e:
                    last_err = e
                    self.log.warn(f"{func.__name__} attempt {i+1}/{max_retries}: {e}")
                except Exception as e:
                    last_err = e
                    self.log.error(f"{func.__name__} attempt {i+1}/{max_retries}: {e}")

                if reconnect:
                    try:
                        self._connect()
                    except Exception as e:
                        self.log.error(f"Reconnect failed: {e}")
                time.sleep(0.5)

            if last_err is not None:
                raise last_err
        return wrapper
    return decorator


class FrankaRobotClient:
    """Robot client for controlling the Franka robot remotely via zerorpc."""

    def __init__(self, server_addr: str = "tcp://192.168.1.7:4242"):
        self.server_addr = server_addr
        self.log = spdlog.ConsoleLogger("FrankaClient")
        self.log.set_level(spdlog.LogLevel.INFO)
        self.client = None
        self._connect()

    def _connect(self):
        self.client = zerorpc.Client(heartbeat=20, timeout=60)
        self.client.connect(self.server_addr)
        self.log.info(f"Connected to {self.server_addr}")

    def close(self):
        if self.client:
            self.client.close()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @robust_call()
    def get_joint_positions(self) -> np.ndarray:
        return np.array(self.client.get_joint_positions())

    @robust_call()
    def get_ee_pose(self) -> Dict[str, np.ndarray]:
        """Returns dict with 't' [x,y,z] and 'q' [qx,qy,qz,qw]."""
        pose = self.client.get_ee_pose()
        return {"t": np.array(pose["t"]), "q": np.array(pose["q"])}

    @robust_call()
    def get_external_torques(self) -> np.ndarray:
        return np.array(self.client.get_external_torques())

    @robust_call()
    def get_torques(self) -> np.ndarray:
        return np.array(self.client.get_torques())

    @robust_call()
    def get_state(self) -> Dict[str, Any]:
        return self.client.get_state()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    @robust_call()
    def move_ee_pose(self, target_ee_pose: List[float],
                     asynchronous: bool = True, delta: bool = True) -> bool:
        """Move EE. target: [x,y,z] | [qx,qy,qz,qw] | [x,y,z,qx,qy,qz,qw]."""
        return self.client.move_ee_pose(target_ee_pose, asynchronous, delta)

    @robust_call()
    def move_joints(self, target_joint_positions: List[float],
                    asynchronous: bool = False) -> bool:
        return self.client.move_joints(target_joint_positions, asynchronous)

    @robust_call()
    def set_joint_impedance(self, impedance: List[float]) -> bool:
        return self.client.set_joint_impedance(impedance)

    @robust_call()
    def start_gravity_compensation(self, duration: float = 3600.0,
                                   joint_impedance: float = 0.0) -> bool:
        return self.client.start_gravity_compensation(duration, joint_impedance)

    @robust_call()
    def is_gravity_compensation_active(self) -> bool:
        return self.client.is_gravity_compensation_active()

    @robust_call()
    def stop_gravity_compensation(self) -> bool:
        return self.client.stop_gravity_compensation()

    @robust_call()
    def stop_motion(self) -> bool:
        return self.client.stop_motion()

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    @robust_call()
    def move_gripper(self, target_width: float, speed: float = 0.1,
                     asynchronous: bool = False) -> bool:
        return self.client.move_gripper(target_width, speed, asynchronous)

    @robust_call()
    def open_gripper(self, speed: float = 0.1, asynchronous: bool = False) -> bool:
        return self.client.open_gripper(speed, asynchronous)

    def release_object(self) -> bool:
        return self.open_gripper()

    @robust_call()
    def grasp_object(self, width: float = 0.0, speed: float = 0.1,
                     force: float = 60.0, epsilon_inner: float = 0.005,
                     epsilon_outer: float = 0.005, asynchronous: bool = False) -> bool:
        return self.client.grasp_object(
            width, speed, force, epsilon_inner, epsilon_outer, asynchronous
        )

    @robust_call()
    def stop_gripper(self, asynchronous: bool = False) -> bool:
        return self.client.stop_gripper(asynchronous)

    @robust_call()
    def homing_gripper(self, asynchronous: bool = False) -> bool:
        return self.client.homing_gripper(asynchronous)

    @robust_call()
    def get_gripper_width(self) -> float:
        return float(self.client.get_gripper_width())

    @robust_call()
    def get_gripper_max_width(self) -> float:
        return float(self.client.get_gripper_max_width())

    @robust_call()
    def is_gripper_grasped(self) -> bool:
        return bool(self.client.is_gripper_grasped())


@click.command()
@click.option("--ip", default="192.168.1.7", help="Robot server IP")
@click.option("--port", default=4242, help="Robot server port")
def main(ip, port):
    """Test connection and print robot state."""
    robot = FrankaRobotClient(f"tcp://{ip}:{port}")
    try:
        print(f"Joints: {robot.get_joint_positions()}")
        print(f"EE Pose: {robot.get_ee_pose()}")
        print(f"Gripper: {robot.get_gripper_width()}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
