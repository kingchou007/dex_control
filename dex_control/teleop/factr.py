"""
FACTR Controller using ROS2 for communication.

Publishes:
- /franka/joint_states (sensor_msgs/JointState)
  - position: joint positions (radians)
  - effort: external torques (Nm)

Subscribes:
- /factr/joint_commands (sensor_msgs/JointState)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from dex_control.robot.robot_client import FrankaRobotClient

class FACTRController(Node):
    def __init__(self, robot_client: FrankaRobotClient, control_rate_hz: float = 30):
        super().__init__('factr_controller')
        
        self.robot_client = robot_client
        self.control_rate_hz = control_rate_hz
        
        # Joint names for FR3 robot (required for ROS2 JointState messages)
        self.joint_names = [
            f'fr3_joint{i}' for i in range(1, 8)
        ]
        
        # QoS Profile - use RELIABLE which handles larger messages better
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher - single topic with all robot state (joint positions and external torques)
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/franka/joint_states', 
            qos_profile
        )
        
        # Subscriber
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/factr/joint_commands',
            self.joint_command_callback,
            qos_profile
        )
        
        # State variables
        self.latest_joint_command = None
        self.command_received = False
        
        # Timer for control loop
        timer_period = 1.0 / control_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'FACTR Controller initialized at {control_rate_hz} Hz')
        self.get_logger().info('Publishing to: /franka/joint_states (positions + external torques)')
        # self.get_logger().info('Subscribing to: /factr/joint_commands')
    
    def joint_command_callback(self, msg: JointState):
        """Callback for joint position commands.
        
        Args:
            msg: JointState message containing target joint positions.
        """
        if len(msg.position) == 7:
            self.latest_joint_command = list(msg.position)
            self.command_received = True
        else:
            self.get_logger().warn(f'Invalid joint command size: {len(msg.position)}, expected 7')
    
    def get_robot_state(self):
        """Get current robot state from the robot client.
        
        Returns:
            tuple: (joint_positions, external_torques) as numpy arrays
        """
        try:
            joint_positions = self.robot_client.get_joint_positions()
            external_torques = self.robot_client.get_external_torques()
            return joint_positions, external_torques
        except Exception as e:
            self.get_logger().error(f'Failed to get robot state: {e}')
            return None, None
    
    def publish_state(self, joint_positions, external_torques):
        """Publish robot state to ROS2 topic.
        
        Args:
            joint_positions: Array of 7 joint positions (radians).
            external_torques: Array of 7 external torques (Nm).
        """
        try:
            # Convert to lists and ensure proper float type
            pos_list = [float(x) for x in joint_positions]
            torque_list = [float(x) for x in external_torques]
            
            # Verify lengths
            if len(pos_list) != 7 or len(torque_list) != 7:
                self.get_logger().error(f'Invalid data length: pos={len(pos_list)}, torque={len(torque_list)}')
                return
            
            # Create message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'fr3_link0'  # Restore frame_id
            msg.name = self.joint_names        # Restore joint names
            msg.position = pos_list
            msg.effort = torque_list
            # No velocity field needed
            
            # Publish
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')
    
    def update_joint_position(self, joint_position):
        """Send joint position command to robot.
        
        Args:
            joint_position: List or array of 7 joint positions (radians).
        """
        
        try:
            # Update only the last 4 joints with values from joint_position, others stay as current
            current_positions = self.robot_client.get_joint_positions()
            safe_command = list(current_positions)
            safe_command[:] = joint_position[:]
            self.robot_client.move_joints(safe_command, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f'Failed to move joints: {e}')


    def control_loop(self):
        """Main control loop called by ROS2 timer."""
        # Get current robot state
        joint_positions, external_torques = self.get_robot_state()
        
        if joint_positions is None or external_torques is None:
            return
        
        # Publish state
        self.publish_state(joint_positions, external_torques)
        
        
        # Execute command if available
        if self.command_received and self.latest_joint_command is not None:
            self.update_joint_position(self.latest_joint_command)
            self.command_received = False  # Clear flag after execution


def main(args=None):
    """Main entry point for FACTR controller."""
    import click
    import time
    @click.command()
    @click.option('--robot-ip', default='192.168.1.7', help='Robot server IP address')
    @click.option('--robot-port', default=4242, help='Robot server port')
    @click.option('--rate', default=30, help='Control loop rate in Hz')
    def run(robot_ip, robot_port, rate):
        """Run FACTR controller with ROS2."""
        # Initialize ROS2
        rclpy.init(args=args)
        time.sleep(1)
        try:
            # Create robot client
            robot_client = FrankaRobotClient(f"tcp://{robot_ip}:{robot_port}")
            
            # Create and run controller
            controller = FACTRController(robot_client, control_rate_hz=rate)
            time.sleep(1)
            
            print(f"FACTR Controller running at {rate} Hz")
            print(f"Connected to robot at {robot_ip}:{robot_port}")
            print("\nROS2 Topics:")
            print("  Publishing:")
            print("    - /franka/joint_states (sensor_msgs/JointState)")
            print("      └─ position: joint positions (rad)")
            print("      └─ effort: external torques (Nm)")
            print("  Subscribing:")
            print("    - /factr/joint_commands (sensor_msgs/JointState)")
            print("\nPress Ctrl+C to stop")
            
            # Spin
            rclpy.spin(controller)
            
        except KeyboardInterrupt:
            print("\nShutting down FACTR controller...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'controller' in locals():
                controller.destroy_node()
            if 'robot_client' in locals():
                robot_client.close()
            rclpy.shutdown()
    
    run()


if __name__ == '__main__':
    main()


