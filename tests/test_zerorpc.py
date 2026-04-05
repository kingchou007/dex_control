"""Test ZeroRPC server/client roundtrip with a fake server object.

No real robot or spdlog/franky needed — uses a plain Python object
to test the full ZeroRPC wiring and client retry logic.

Run:
    pytest tests/test_zerorpc.py -v
"""

import time

import gevent
import numpy as np
import pytest
import zerorpc

from dex_control.robot.robot_client import FrankaRobotClient


# ---------------------------------------------------------------------------
# Mock robot (plain Python object — ZeroRPC exposes all public methods)
# ---------------------------------------------------------------------------

_JOINTS = [0.0, -0.3, 0.0, -2.5, 0.0, 2.3, 0.8]
_EE_T = [0.55, 0.0, 0.4]
_EE_Q = [0.0, 0.0, 1.0, 0.0]
_TORQUES = [0.1, 0.2, 0.0, -0.1, 0.0, 0.0, 0.0]


class MockRobot:
    """Minimal robot object that returns canned data."""

    def get_joint_positions(self):
        return _JOINTS

    def get_ee_pose(self):
        return {"t": _EE_T, "q": _EE_Q}

    def get_external_torques(self):
        return _TORQUES

    def get_state(self):
        return {"q": _JOINTS, "dq": [0.0] * 7}

    def move_ee_pose(self, target_pose, asynchronous=True, delta=True):
        return True

    def move_ee_waypoints(self, waypoints, delta=True):
        return True

    def move_joints(self, target_positions, asynchronous=False):
        return True

    def move_joint_waypoints(self, waypoints):
        return True

    def get_gripper_width(self):
        return 0.04

    def get_gripper_max_width(self):
        return 0.08

    def is_gripper_grasped(self):
        return False

    def move_gripper(self, target_width, speed=0.1, asynchronous=False):
        return True

    def open_gripper(self, speed=0.1, asynchronous=False):
        return True

    def grasp_object(self, width=0.0, speed=0.1, force=60.0,
                     epsilon_inner=0.005, epsilon_outer=0.005, asynchronous=False):
        return True

    def stop_gripper(self, asynchronous=False):
        return True

    def homing_gripper(self, asynchronous=False):
        return True

    def set_joint_impedance(self, impedance):
        return True

    def start_gravity_compensation(self, duration=3600.0):
        return True

    def stop_motion(self):
        return True


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _free_port():
    """Find a free TCP port."""
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def server_and_client():
    """Start a local ZeroRPC server with MockRobot and return client."""
    port = _free_port()
    addr = f"tcp://127.0.0.1:{port}"

    server = zerorpc.Server(MockRobot(), heartbeat=None)
    server.bind(addr)

    greenlet = gevent.spawn(server.run)
    gevent.sleep(0.3)  # let server start

    client = FrankaRobotClient(addr)
    yield client

    client.close()
    server.close()
    greenlet.kill()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestStateQueries:
    def test_get_joint_positions(self, server_and_client):
        joints = server_and_client.get_joint_positions()
        assert isinstance(joints, np.ndarray)
        assert len(joints) == 7
        np.testing.assert_allclose(joints, _JOINTS)

    def test_get_ee_pose(self, server_and_client):
        pose = server_and_client.get_ee_pose()
        assert "t" in pose and "q" in pose
        np.testing.assert_allclose(pose["t"], _EE_T)
        np.testing.assert_allclose(pose["q"], _EE_Q)

    def test_get_external_torques(self, server_and_client):
        torques = server_and_client.get_external_torques()
        assert isinstance(torques, np.ndarray)
        np.testing.assert_allclose(torques, _TORQUES)

    def test_get_state(self, server_and_client):
        state = server_and_client.get_state()
        assert "q" in state and "dq" in state
        assert len(state["q"]) == 7


class TestMotionCommands:
    def test_move_ee_pose(self, server_and_client):
        ok = server_and_client.move_ee_pose([0.1, 0.0, 0.0], delta=True)
        assert ok is True

    def test_move_joints(self, server_and_client):
        ok = server_and_client.move_joints(_JOINTS, asynchronous=False)
        assert ok is True

    def test_move_ee_waypoints(self, server_and_client):
        wps = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0]]
        ok = server_and_client.move_ee_waypoints(wps, delta=True)
        assert ok is True

    def test_move_joint_waypoints(self, server_and_client):
        ok = server_and_client.move_joint_waypoints([_JOINTS, _JOINTS])
        assert ok is True


class TestGripper:
    def test_get_gripper_width(self, server_and_client):
        assert server_and_client.get_gripper_width() == pytest.approx(0.04)

    def test_get_gripper_max_width(self, server_and_client):
        assert server_and_client.get_gripper_max_width() == pytest.approx(0.08)

    def test_is_gripper_grasped(self, server_and_client):
        assert server_and_client.is_gripper_grasped() is False

    def test_open_gripper(self, server_and_client):
        assert server_and_client.open_gripper(speed=0.05) is True

    def test_grasp_object(self, server_and_client):
        ok = server_and_client.grasp_object(width=0.03, speed=0.1, force=40.0)
        assert ok is True

    def test_move_gripper(self, server_and_client):
        assert server_and_client.move_gripper(0.05) is True

    def test_stop_gripper(self, server_and_client):
        assert server_and_client.stop_gripper() is True

    def test_homing_gripper(self, server_and_client):
        assert server_and_client.homing_gripper() is True

    def test_release_object_alias(self, server_and_client):
        assert server_and_client.release_object() is True


class TestImpedance:
    def test_set_joint_impedance(self, server_and_client):
        assert server_and_client.set_joint_impedance([50.0] * 7) is True

    def test_start_gravity_compensation(self, server_and_client):
        assert server_and_client.start_gravity_compensation(duration=60.0) is True

    def test_stop_motion(self, server_and_client):
        assert server_and_client.stop_motion() is True
