"""ZMQ PUB/SUB for streaming robot state without ROS2.

Publisher side (teleop):
    pub = ZMQStatePublisher(port=5557)
    pub.publish(ee_pose, joint_positions, gripper_width)

Subscriber side (data collection):
    sub = ZMQStateSubscriber(port=5557)
    state = sub.receive()  # blocking, or receive(timeout=10) for non-blocking
"""

import json
import time
import zmq
import numpy as np


class ZMQStatePublisher:
    """Publishes robot state over ZMQ PUB socket."""

    def __init__(self, port: int = 5557):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        self.port = port

    def publish(self, ee_pose: dict, joint_positions, gripper_width: float = None):
        """Publish robot state as JSON.

        Args:
            ee_pose: dict with 't' (xyz) and 'q' (quaternion) keys
            joint_positions: array-like of joint angles
            gripper_width: optional gripper width
        """
        msg = {
            "timestamp": time.time(),
            "ee_pose": {
                "t": [float(x) for x in ee_pose["t"]],
                "q": [float(x) for x in ee_pose["q"]],
            },
            "joint_positions": [float(x) for x in joint_positions],
        }
        if gripper_width is not None:
            msg["gripper_width"] = float(gripper_width)

        self.socket.send_string(json.dumps(msg))

    def close(self):
        self.socket.close()
        self.context.term()


class ZMQStateSubscriber:
    """Subscribes to robot state over ZMQ SUB socket."""

    def __init__(self, host: str = "localhost", port: int = 5557):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

    def receive(self, timeout: int = None):
        """Receive robot state. Returns dict or None if timeout.

        Args:
            timeout: timeout in ms. None = blocking.
        """
        if timeout is not None:
            self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        else:
            self.socket.setsockopt(zmq.RCVTIMEO, -1)

        try:
            msg = self.socket.recv_string()
            return json.loads(msg)
        except zmq.Again:
            return None

    def close(self):
        self.socket.close()
        self.context.term()
