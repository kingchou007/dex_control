"""
Action ZMQ Server.
- Receives action commands from eval.py via ZMQ
- Controls robot directly via FrankaRobotClient
- Non-blocking ZMQ

Usage:
    python action_zmq_server.py
"""

import time
from typing import Dict, Any, Optional
import zmq
import click
import numpy as np

from dex_control.robot.robot_client import FrankaRobotClient
from termcolor import cprint


DEFAULT_IP = "192.168.1.6"
DEFAULT_PORT = 4242
ZMQ_PORT_DEFAULT = 5556


class ActionZMQServer:
    """
    Standalone ZMQ Server for robot control.
    """

    def __init__(self, robot: FrankaRobotClient, zmq_port: int = ZMQ_PORT_DEFAULT):
        """
        Initialize the ActionZMQServer.

        Args:
            robot: Connected instance of FrankaRobotClient.
            zmq_port: Port to bind the ZMQ REP socket to.
        """
        self.robot = robot

        # ZMQ setup (non-blocking)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{zmq_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout for non-blocking

        cprint(f"Action server started on ZMQ port {zmq_port}", "green")

    def process_zmq(self) -> bool:
        """
        Process incoming ZMQ messages in a non-blocking manner.

        Returns:
            bool: True if a 'stop' command was received, False otherwise.
        """
        try:
            msg: Dict[str, Any] = self.socket.recv_json(zmq.NOBLOCK)
            cmd: str = msg.get("cmd", "")

            if cmd == "set_pose":
                pose = np.array(msg["pose"])
                # pose format: [x, y, z, qx, qy, qz, qw]
                target_pose = pose.tolist()
                self.robot.move_ee_pose(target_pose, asynchronous=True, delta=False)
                self.socket.send_json({"status": "ok"})
                cprint(f"Moved to: {pose[:3]}", "cyan")

            elif cmd == "get_pose":
                ee_pose = self.robot.get_ee_pose()
                pose = [
                    ee_pose["t"][0], ee_pose["t"][1], ee_pose["t"][2],
                    ee_pose["q"][0], ee_pose["q"][1], ee_pose["q"][2], ee_pose["q"][3]
                ]
                self.socket.send_json({"status": "ok", "pose": pose})

            elif cmd == "stop":
                self.socket.send_json({"status": "ok"})
                return True

            else:
                self.socket.send_json({"status": "error", "msg": f"Unknown command: {cmd}"})

        except zmq.Again:
            pass  # No message, continue
        except Exception as e:
            cprint(f"ZMQ Error: {e}", "red")
            try:
                self.socket.send_json({"status": "error", "msg": str(e)})
            except Exception:
                pass

        return False

    def close(self) -> None:
        """Clean up ZMQ resources."""
        self.socket.close()
        self.context.term()


@click.command()
@click.option("--ip", default=DEFAULT_IP, help="Robot server IP address")
@click.option("--port", default=DEFAULT_PORT, help="Robot server port")
@click.option("--zmq-port", default=ZMQ_PORT_DEFAULT, help="ZMQ port for action commands")
def main(ip: str, port: int, zmq_port: int) -> None:
    """Action ZMQ Server."""

    server_addr = f"tcp://{ip}:{port}"

    try:
        cprint(f"Connecting to robot at {server_addr}...", "cyan")
        robot = FrankaRobotClient(server_addr=server_addr)
        cprint("Robot connected!", "green")
    except Exception as e:
        cprint(f"Error connecting to robot: {e}", "red")
        return

    server = ActionZMQServer(robot=robot, zmq_port=zmq_port)

    try:
        while True:
            # Process ZMQ (non-blocking)
            should_stop = server.process_zmq()
            if should_stop:
                break
            time.sleep(0.001)

    except KeyboardInterrupt:
        cprint("\nShutting down...", "yellow")
    finally:
        server.close()


if __name__ == "__main__":
    main()
