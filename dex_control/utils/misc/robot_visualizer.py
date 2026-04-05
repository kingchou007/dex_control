"""Franka robot visualizer using viser (web-based 3D, like rviz but ROS-free).

Usage as standalone:
    python -m dex_control.utils.misc.robot_visualizer --ip 192.168.1.6

Usage from teleop (plug-in):
    from dex_control.utils.misc.robot_visualizer import RobotVisualizer

    vis = RobotVisualizer()  # opens browser at http://localhost:8080
    # in your control loop:
    vis.update(joint_positions, ee_pose, gripper_width)
    # when done:
    vis.close()

Or subscribe to ZMQ state stream:
    python -m dex_control.utils.misc.robot_visualizer --zmq-port 5557
"""

import time
import threading
import numpy as np
from pathlib import Path
from typing import Optional

import viser
import viser.transforms as vtf
from viser.extras import ViserUrdf


# FR3 joint names in URDF order
FR3_JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]

# Default URDF path (relative to repo root)
DEFAULT_URDF_PATH = Path(__file__).parent.parent.parent.parent / "assets" / "fr3" / "fr3.urdf"


class RobotVisualizer:
    """Real-time Franka robot visualization via viser web UI."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080,
                 urdf_path: Optional[str] = None):
        self.server = viser.ViserServer(host=host, port=port)

        # Load URDF
        urdf_path = Path(urdf_path) if urdf_path else DEFAULT_URDF_PATH
        self.viser_urdf = ViserUrdf(self.server, urdf_path)

        # Add ground grid
        self.server.scene.add_grid("/grid", width=2.0, height=2.0, cell_size=0.1)

        # Add EE frame visualization
        self.ee_frame = self.server.scene.add_frame(
            "/ee_frame", axes_length=0.1, axes_radius=0.005
        )

        # GUI elements
        with self.server.gui.add_folder("Robot State"):
            self.gui_joint_text = self.server.gui.add_text(
                "Joints", initial_value="--", disabled=True
            )
            self.gui_ee_text = self.server.gui.add_text(
                "EE Pose", initial_value="--", disabled=True
            )
            self.gui_gripper_text = self.server.gui.add_text(
                "Gripper", initial_value="--", disabled=True
            )

        print(f"[Visualizer] Open http://localhost:{port} in your browser")

    def update(self, joint_positions, ee_pose: Optional[dict] = None,
               gripper_width: Optional[float] = None):
        """Update robot visualization.

        Args:
            joint_positions: array of 7 joint angles (radians)
            ee_pose: dict with 't' [x,y,z] and 'q' [qx,qy,qz,qw]
            gripper_width: gripper width in meters
        """
        # Update URDF joint config
        joint_config = {
            name: float(joint_positions[i])
            for i, name in enumerate(FR3_JOINT_NAMES)
        }
        self.viser_urdf.update_cfg(joint_config)

        # Update EE frame
        if ee_pose is not None:
            t = ee_pose["t"]
            q = ee_pose["q"]  # [qx, qy, qz, qw]
            self.ee_frame.position = (t[0], t[1], t[2])
            self.ee_frame.wxyz = (q[3], q[0], q[1], q[2])  # viser uses wxyz

        # Update GUI text
        if joint_positions is not None:
            joint_str = ", ".join([f"{j:.2f}" for j in joint_positions])
            self.gui_joint_text.value = f"[{joint_str}]"

        if ee_pose is not None:
            t = ee_pose["t"]
            self.gui_ee_text.value = f"[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}]"

        if gripper_width is not None:
            self.gui_gripper_text.value = f"{gripper_width:.4f} m"

    def close(self):
        """Shutdown the viser server."""
        pass  # viser server cleans up on process exit


def run_with_zmq(host, port, zmq_host, zmq_port, urdf_path):
    """Run visualizer subscribing to ZMQ state stream."""
    from dex_control.utils.zmq_publisher import ZMQStateSubscriber

    vis = RobotVisualizer(host=host, port=port, urdf_path=urdf_path)
    sub = ZMQStateSubscriber(host=zmq_host, port=zmq_port)
    print(f"[Visualizer] Subscribing to ZMQ {zmq_host}:{zmq_port}")

    try:
        while True:
            state = sub.receive(timeout=100)
            if state is None:
                continue
            ee_pose = state.get("ee_pose")
            joints = np.array(state.get("joint_positions", [0.0] * 7))
            gripper = state.get("gripper_width")
            vis.update(joints, ee_pose, gripper)
    except KeyboardInterrupt:
        print("\n[Visualizer] Shutting down...")
    finally:
        sub.close()


def run_with_robot(host, port, robot_ip, robot_port, urdf_path, rate):
    """Run visualizer connected directly to robot."""
    from dex_control.robot.robot_client import FrankaRobotClient
    from termcolor import cprint

    vis = RobotVisualizer(host=host, port=port, urdf_path=urdf_path)

    server_addr = f"tcp://{robot_ip}:{robot_port}"
    cprint(f"Connecting to robot at {server_addr}...", "cyan")
    robot = FrankaRobotClient(server_addr=server_addr)
    cprint("Connected!", "green")

    dt = 1.0 / rate
    try:
        while True:
            joints = robot.get_joint_positions()
            ee_pose = robot.get_ee_pose()
            try:
                gripper_width = robot.get_gripper_width()
            except Exception:
                gripper_width = None
            vis.update(joints, ee_pose, gripper_width)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[Visualizer] Shutting down...")


if __name__ == "__main__":
    import click

    @click.command()
    @click.option("--host", default="0.0.0.0", help="Viser server host")
    @click.option("--port", default=8080, help="Viser server port")
    @click.option("--mode", default="zmq", type=click.Choice(["zmq", "robot"]),
                  help="Data source: zmq (subscribe) or robot (direct)")
    @click.option("--ip", default="192.168.1.6", help="Robot IP (robot mode)")
    @click.option("--robot-port", default=4242, help="Robot port (robot mode)")
    @click.option("--zmq-host", default="localhost", help="ZMQ host (zmq mode)")
    @click.option("--zmq-port", default=5557, help="ZMQ port (zmq mode)")
    @click.option("--urdf", default=None, help="Path to URDF file")
    @click.option("--rate", default=30.0, help="Update rate Hz (robot mode)")
    def main(host, port, mode, ip, robot_port, zmq_host, zmq_port, urdf, rate):
        """Franka robot visualizer (viser web UI)."""
        if mode == "zmq":
            run_with_zmq(host, port, zmq_host, zmq_port, urdf)
        else:
            run_with_robot(host, port, ip, robot_port, urdf, rate)

    main()
