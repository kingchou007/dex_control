"""Test gRPC server/client roundtrip with a fake servicer.

No real robot or spdlog/franky needed — uses a standalone mock servicer
to test the full gRPC wiring and client retry logic.

Run:
    pytest tests/test_grpc.py -v
"""

from concurrent import futures

import grpc
import numpy as np
import pytest

from dex_control.proto import franka_pb2, franka_pb2_grpc
from dex_control.robot.robot_client import FrankaRobotClient


# ---------------------------------------------------------------------------
# Mock servicer (avoids importing franka_wrapper which needs spdlog/franky)
# ---------------------------------------------------------------------------

# Fake joint values used across tests
_JOINTS = [0.0, -0.3, 0.0, -2.5, 0.0, 2.3, 0.8]
_EE_T = [0.55, 0.0, 0.4]
_EE_Q = [0.0, 0.0, 1.0, 0.0]
_TORQUES = [0.1, 0.2, 0.0, -0.1, 0.0, 0.0, 0.0]


class MockServicer(franka_pb2_grpc.FrankaRobotServicer):
    """Minimal servicer that returns canned data and records calls."""

    def __init__(self):
        self.calls = []  # list of (method_name, request) tuples

    def _record(self, name, request):
        self.calls[len(self.calls):] = [(name, request)]

    # -- State --
    def GetJointPositions(self, request, context):
        self._record("GetJointPositions", request)
        return franka_pb2.JointValues(values=_JOINTS)

    def GetEePose(self, request, context):
        self._record("GetEePose", request)
        return franka_pb2.EePose(translation=_EE_T, quaternion=_EE_Q)

    def GetExternalTorques(self, request, context):
        self._record("GetExternalTorques", request)
        return franka_pb2.JointValues(values=_TORQUES)

    def GetState(self, request, context):
        self._record("GetState", request)
        data = {
            "q": franka_pb2.DoubleList(values=_JOINTS),
            "dq": franka_pb2.DoubleList(values=[0.0] * 7),
        }
        return franka_pb2.RobotState(data=data)

    # -- Motion --
    def MoveEePose(self, request, context):
        self._record("MoveEePose", request)
        return franka_pb2.BoolResponse(success=True)

    def MoveEeWaypoints(self, request, context):
        self._record("MoveEeWaypoints", request)
        return franka_pb2.BoolResponse(success=True)

    def MoveJoints(self, request, context):
        self._record("MoveJoints", request)
        return franka_pb2.BoolResponse(success=True)

    def MoveJointWaypoints(self, request, context):
        self._record("MoveJointWaypoints", request)
        return franka_pb2.BoolResponse(success=True)

    # -- Gripper state --
    def GetGripperWidth(self, request, context):
        return franka_pb2.FloatResponse(value=0.04)

    def GetGripperMaxWidth(self, request, context):
        return franka_pb2.FloatResponse(value=0.08)

    def IsGripperGrasped(self, request, context):
        return franka_pb2.BoolResponse(success=False)

    # -- Gripper control --
    def MoveGripper(self, request, context):
        self._record("MoveGripper", request)
        return franka_pb2.BoolResponse(success=True)

    def OpenGripper(self, request, context):
        self._record("OpenGripper", request)
        return franka_pb2.BoolResponse(success=True)

    def GraspObject(self, request, context):
        self._record("GraspObject", request)
        return franka_pb2.BoolResponse(success=True)

    def StopGripper(self, request, context):
        return franka_pb2.BoolResponse(success=True)

    def HomingGripper(self, request, context):
        return franka_pb2.BoolResponse(success=True)

    # -- Impedance --
    def SetJointImpedance(self, request, context):
        self._record("SetJointImpedance", request)
        return franka_pb2.BoolResponse(success=True)

    def StartGravityCompensation(self, request, context):
        self._record("StartGravityCompensation", request)
        return franka_pb2.BoolResponse(success=True)

    def StopMotion(self, request, context):
        return franka_pb2.BoolResponse(success=True)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def server_and_client():
    """Start a local gRPC server with MockServicer and return (client, servicer)."""
    servicer = MockServicer()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    franka_pb2_grpc.add_FrankaRobotServicer_to_server(servicer, server)
    port = server.add_insecure_port("localhost:0")
    server.start()

    client = FrankaRobotClient(f"localhost:{port}", max_retries=1)
    yield client, servicer

    client.close()
    server.stop(grace=0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestStateQueries:
    def test_get_joint_positions(self, server_and_client):
        client, _ = server_and_client
        joints = client.get_joint_positions()
        assert isinstance(joints, np.ndarray)
        assert len(joints) == 7
        np.testing.assert_allclose(joints, _JOINTS)

    def test_get_ee_pose(self, server_and_client):
        client, _ = server_and_client
        pose = client.get_ee_pose()
        assert "t" in pose and "q" in pose
        np.testing.assert_allclose(pose["t"], _EE_T)
        np.testing.assert_allclose(pose["q"], _EE_Q)

    def test_get_external_torques(self, server_and_client):
        client, _ = server_and_client
        torques = client.get_external_torques()
        assert isinstance(torques, np.ndarray)
        np.testing.assert_allclose(torques, _TORQUES)

    def test_get_state(self, server_and_client):
        client, _ = server_and_client
        state = client.get_state()
        assert "q" in state and "dq" in state
        assert len(state["q"]) == 7
        assert state["dq"] == [0.0] * 7


class TestMotionCommands:
    def test_move_ee_pose_relative(self, server_and_client):
        client, svc = server_and_client
        ok = client.move_ee_pose([0.1, 0.0, 0.0], delta=True)
        assert ok is True
        # Check the servicer received correct args
        name, req = svc.calls[-1]
        assert name == "MoveEePose"
        np.testing.assert_allclose(list(req.target_pose), [0.1, 0.0, 0.0])
        assert req.delta is True

    def test_move_ee_pose_absolute_7d(self, server_and_client):
        client, svc = server_and_client
        pose = [0.55, 0.0, 0.4, 0.0, 0.0, 1.0, 0.0]
        ok = client.move_ee_pose(pose, asynchronous=False, delta=False)
        assert ok is True
        _, req = svc.calls[-1]
        np.testing.assert_allclose(list(req.target_pose), pose)
        assert req.asynchronous is False
        assert req.delta is False

    def test_move_joints(self, server_and_client):
        client, svc = server_and_client
        ok = client.move_joints(_JOINTS, asynchronous=False)
        assert ok is True
        _, req = svc.calls[-1]
        np.testing.assert_allclose(list(req.target_positions), _JOINTS)

    def test_move_ee_waypoints(self, server_and_client):
        client, _ = server_and_client
        wps = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0]]
        ok = client.move_ee_waypoints(wps, delta=True)
        assert ok is True

    def test_move_joint_waypoints(self, server_and_client):
        client, _ = server_and_client
        wps = [_JOINTS, _JOINTS]
        ok = client.move_joint_waypoints(wps)
        assert ok is True


class TestGripper:
    def test_get_gripper_width(self, server_and_client):
        client, _ = server_and_client
        assert client.get_gripper_width() == pytest.approx(0.04)

    def test_get_gripper_max_width(self, server_and_client):
        client, _ = server_and_client
        assert client.get_gripper_max_width() == pytest.approx(0.08)

    def test_is_gripper_grasped(self, server_and_client):
        client, _ = server_and_client
        assert client.is_gripper_grasped() is False

    def test_open_gripper(self, server_and_client):
        client, svc = server_and_client
        assert client.open_gripper(speed=0.05) is True
        _, req = svc.calls[-1]
        assert req.speed == pytest.approx(0.05)

    def test_grasp_object(self, server_and_client):
        client, svc = server_and_client
        ok = client.grasp_object(width=0.03, speed=0.1, force=40.0)
        assert ok is True
        _, req = svc.calls[-1]
        assert req.width == pytest.approx(0.03)
        assert req.force == pytest.approx(40.0)

    def test_move_gripper(self, server_and_client):
        client, _ = server_and_client
        assert client.move_gripper(0.05) is True

    def test_stop_gripper(self, server_and_client):
        client, _ = server_and_client
        assert client.stop_gripper() is True

    def test_homing_gripper(self, server_and_client):
        client, _ = server_and_client
        assert client.homing_gripper() is True

    def test_release_object_alias(self, server_and_client):
        client, _ = server_and_client
        assert client.release_object() is True


class TestImpedance:
    def test_set_joint_impedance(self, server_and_client):
        client, svc = server_and_client
        impedance = [50.0] * 7
        assert client.set_joint_impedance(impedance) is True
        _, req = svc.calls[-1]
        assert list(req.values) == impedance

    def test_start_gravity_compensation(self, server_and_client):
        client, svc = server_and_client
        assert client.start_gravity_compensation(duration=60.0) is True
        _, req = svc.calls[-1]
        assert req.duration == pytest.approx(60.0)

    def test_stop_motion(self, server_and_client):
        client, _ = server_and_client
        assert client.stop_motion() is True


class TestRetry:
    def test_retry_on_transient_failure(self):
        """Client retries on UNAVAILABLE and succeeds on second attempt."""
        call_count = 0

        class FlakeyServicer(franka_pb2_grpc.FrankaRobotServicer):
            def GetJointPositions(self, request, context):
                nonlocal call_count
                call_count += 1
                if call_count == 1:
                    context.abort(grpc.StatusCode.UNAVAILABLE, "transient")
                return franka_pb2.JointValues(values=[0.0] * 7)

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
        franka_pb2_grpc.add_FrankaRobotServicer_to_server(FlakeyServicer(), server)
        port = server.add_insecure_port("localhost:0")
        server.start()

        try:
            client = FrankaRobotClient(f"localhost:{port}", max_retries=3)
            joints = client.get_joint_positions()
            assert len(joints) == 7
            assert call_count == 2
            client.close()
        finally:
            server.stop(grace=0)

    def test_raises_after_max_retries(self):
        """Client raises after exhausting all retry attempts."""

        class AlwaysFailServicer(franka_pb2_grpc.FrankaRobotServicer):
            def GetJointPositions(self, request, context):
                context.abort(grpc.StatusCode.INTERNAL, "permanent error")

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
        franka_pb2_grpc.add_FrankaRobotServicer_to_server(AlwaysFailServicer(), server)
        port = server.add_insecure_port("localhost:0")
        server.start()

        try:
            client = FrankaRobotClient(f"localhost:{port}", max_retries=2)
            with pytest.raises(grpc.RpcError):
                client.get_joint_positions()
            client.close()
        finally:
            server.stop(grace=0)
