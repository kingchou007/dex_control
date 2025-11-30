import time
import asyncio
import numpy as np
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
from scipy.spatial.transform import Rotation as R
from typing import Optional
import zerorpc


class FrankaWrapper:
    """
    High-level wrapper for Franka robot control.
    
    Provides a simplified interface for common robot operations:
    - Moving end-effector by translation/rotation (delta or absolute)
    - Moving joints to target positions 
    - Getting current robot state 
    - All control is non-blocking and asynchronous, so it's implemented in an event-driven manner.
    
    This wrapper manages the underlying FrankaController and automatically
    switches between control modes as needed.

    # We temporarily use the aiofranka library for the controller, 
    # but we will switch to the our own lab controller library in the future.
    
    Attributes:
        robot (RobotInterface): Low-level robot interface
        controller (FrankaController): High-level controller
        ip (str): Robot IP address
        started (bool): Whether controller is started
    """
    
    def __init__(self, ip: str, controller_type: str = "osc"):
        """
        Initialize robot wrapper.
        
        Args:
            ip (str): Robot IP address (e.g., "192.168.1.33")
        """
        self.ip = ip
        self.robot = RobotInterface(ip)
        self.controller = FrankaController(self.robot)
        self.started = False

        self.controller_type = controller_type
        self.controller.switch(self.controller_type)
        
        # TODO: make this configurable
        # Default OSC gains for end-effector control
        self.default_ee_kp = np.array([300.0, 300.0, 300.0, 1000.0, 1000.0, 1000.0])
        self.default_ee_kd = np.ones(6) * 10.0
        
        # Default update frequency
        self.default_freq = 100.0
    
    async def start(self):
        """
        Start the robot controller.
        
        Must be called before using move_ee_pose() or move_joints().
        Moves robot to home position after starting.
        """
        await self.controller.start()
        await self.controller.move()  # Move to home position
        await asyncio.sleep(1.0)      # Wait for motion to complete
        self.started = True
    
    async def stop(self):
        """ Stop the robot controller."""
        await self.controller.stop()
        self.started = False
    
    def get_state(self) -> dict:
        """ Get current robot state. """
        return self.controller.robot.state
    
    async def move_ee_pose(
        self, 
        translation: np.ndarray, 
        rotation: np.ndarray,
        delta: bool = False
    ):
        """
        Move end-effector by translation and rotation. """

        # Ensure inputs are numpy arrays
        translation = np.array(translation)
        rotation = np.array(rotation)
        
        # Convert rotation to rotation matrix if needed
        if rotation.shape == (3, 3):
            rot_matrix = rotation
        elif rotation.shape == (4,):
            # Quaternion [x, y, z, w]
            rot_matrix = R.from_quat(rotation).as_matrix()
        else:
            raise ValueError(f"Invalid rotation shape: {rotation.shape}. Expected (3,3) or (4,)")
        
        # Get current end-effector pose
        current_state = self.controller.robot.state
        current_ee = current_state['ee'].copy()
        
        # Apply delta or absolute positioning
        if delta:
            # Relative: add translation, compose rotations
            current_ee[:3, 3] += translation
            current_ee[:3, :3] = rot_matrix @ current_ee[:3, :3]
        else:
            # Absolute: set directly
            current_ee[:3, 3] = translation
            current_ee[:3, :3] = rot_matrix
        
        await self.controller.set("ee_desired", current_ee)
    
    async def move_joints(
        self, 
        joint_positions: np.ndarray,
        use_trajectory: bool = True,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        freq: Optional[float] = None
    ):
        """
        Move robot joints to target positions.
        It's non-blocking and asynchronous, so it's implemented in an event-driven manner.
        """
     
        joint_positions = np.array(joint_positions)
        await self.controller.move(joint_positions.tolist())
     

if __name__ == "__main__":
    # Start ZeroRPC server for remote control
    ip = "192.168.1.33"
    server = FrankaWrapper(ip)
    
    # Start the robot controller
    print("Starting robot controller...")
    asyncio.run(server.start())
    print("Robot controller started and moved to home position")
    
    # Start ZeroRPC server
    s = zerorpc.Server(server, heartbeat=None)
    s.bind("tcp://0.0.0.0:4242")
    print("FrankaServer listening on tcp://0.0.0.0:4242")
    s.run()

