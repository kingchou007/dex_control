"""Real-time Franka robot visualizer

Visualize real Franka robot using joint positions from the robot control system.

Requires yourdfpy and robot_descriptions. Any URDF supported by yourdfpy should work.

- https://github.com/robot-descriptions/robot_descriptions.py
- https://github.com/clemense/yourdfpy

**Features:**

* :class:`viser.extras.ViserUrdf` for URDF file parsing and visualization
* Real-time robot pose updates from actual robot
* Support for local URDF files and robot_descriptions library
"""

# This is futrure feature we would like to add

from __future__ import annotations

import time
from typing import Literal

import numpy as np
import tyro
from robot_descriptions.loaders.yourdfpy import load_robot_description

import viser
from viser.extras import ViserUrdf

# Import robot client to get real joint positions
import sys
from pathlib import Path

# Add parent directory to path to import dex_control
sys.path.insert(0, str(Path(__file__).parent.parent))
from dex_control.robot.client import NucRobotClient


def get_joint_order_from_urdf(viser_urdf: ViserUrdf) -> list[str]:
    """Get the order of actuated joints from URDF."""
    return list(viser_urdf.get_actuated_joint_limits().keys())


def main(
    robot_type: Literal[
        "panda",
        "ur10",
        "cassie",
        "allegro_hand",
        "barrett_hand",
        "robotiq_2f85",
        "atlas_drc",
        "g1",
        "h1",
        "anymal_c",
        "go2",
    ] = "panda",
    load_meshes: bool = True,
    load_collision_meshes: bool = False,
    nuc_addr: str = "tcp://192.168.1.7:4242",
    update_hz: float = 30.0,
) -> None:
    # Connect to real robot
    print(f"Connecting to robot at {nuc_addr}...")
    robot_client = NucRobotClient(nuc_addr)
    print("Connected to robot!")

    # Start viser server.
    server = viser.ViserServer()

    # Load URDF.
    #
    # This takes either a yourdfpy.URDF object or a path to a .urdf file.
    urdf = load_robot_description(
        robot_type + "_description",
        load_meshes=load_meshes,
        build_scene_graph=load_meshes,
        load_collision_meshes=load_collision_meshes,
        build_collision_scene_graph=load_collision_meshes,
    )
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf,
        load_meshes=load_meshes,
        load_collision_meshes=load_collision_meshes,
        collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
    )

    # Get joint order from URDF (should match Franka's joint order)
    joint_order = get_joint_order_from_urdf(viser_urdf)
    print(f"URDF joint order: {joint_order}")
    
    # Franka Panda has 7 joints, verify we have the right number
    if len(joint_order) != 7:
        print(f"Warning: Expected 7 joints for Franka Panda, got {len(joint_order)}")
    
    # Get initial configuration from robot
    initial_state = robot_client.get_state()
    initial_config = initial_state["joint_positions"]
    print(f"Initial joint positions: {initial_config}")

    # Add visibility checkboxes.
    with server.gui.add_folder("Visibility"):
        show_meshes_cb = server.gui.add_checkbox(
            "Show meshes",
            viser_urdf.show_visual,
        )
        show_collision_meshes_cb = server.gui.add_checkbox(
            "Show collision meshes", viser_urdf.show_collision
        )

    @show_meshes_cb.on_update
    def _(_):
        viser_urdf.show_visual = show_meshes_cb.value

    @show_collision_meshes_cb.on_update
    def _(_):
        viser_urdf.show_collision = show_collision_meshes_cb.value

    # Hide checkboxes if meshes are not loaded.
    show_meshes_cb.visible = load_meshes
    show_collision_meshes_cb.visible = load_collision_meshes

    # Set initial robot configuration.
    viser_urdf.update_cfg(np.array(initial_config))

    # Create grid.
    trimesh_scene = viser_urdf._urdf.scene or viser_urdf._urdf.collision_scene
    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(
            0.0,
            0.0,
            # Get the minimum z value of the trimesh scene.
            trimesh_scene.bounds[0, 2] if trimesh_scene is not None else 0.0,
        ),
    )

    # Add status text showing current joint positions
    status_text = server.gui.add_text("Status: Connected", initial_value="Status: Connected")

    # Real-time update loop
    dt = 1.0 / update_hz
    print(f"Starting visualization update loop at {update_hz} Hz...")
    
    while True:
        try:
            # Get current robot state
            state = robot_client.get_state()
            joint_positions = np.array(state["joint_positions"])
            
            # Update URDF visualization with real joint positions
            viser_urdf.update_cfg(joint_positions)
            
            # Update status text
            status_text.value = f"Joint positions: {[f'{q:.3f}' for q in joint_positions]}"
            
            # Sleep to maintain update rate
            time.sleep(dt)
        except KeyboardInterrupt:
            print("\nStopping visualization...")
            break
        except Exception as e:
            print(f"Error updating robot state: {e}")
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
