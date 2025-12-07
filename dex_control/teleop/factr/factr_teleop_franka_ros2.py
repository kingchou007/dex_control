# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------


import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from factr_teleop.factr_teleop import FACTRTeleop


def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


class FACTRTeleopFrankaZMQ(FACTRTeleop):
    """
    FACTR teleoperation bridge that now communicates with the Franka follower arm purely through
    ROS 2 topics. Concretely, this node:

      * Publishes `/factr/joint_commands` (sensor_msgs/JointState). The `position` vector stores the
        leader-side (Dynamixel) joint angles that the Franka follower should track.
      * Subscribes to `/franka/joint_states` (sensor_msgs/JointState) to obtain the follower's joint
        positions (and, if provided, `effort` for external torques).
      * Optionally subscribes to `/franka/joint_torques` when external torques are exposed on a
        dedicated topic, so that leader-side force feedback can be enabled.

    This class continues to re-publish additional ROS topics for logging/debugging. The gripper is
    not commanded anymore—the follower side only tracks the seven arm joints—though leader-side
    gripper feedback can still be enabled if desired.
    """

    def __init__(self):
        self.latest_franka_state = None
        self.latest_franka_torque = None
        self._missing_torque_warning_issued = False
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0

    def set_up_communication(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ROS publisher for sending joint position targets to the follower Franka arm
        self.factr_cmd_pub = self.create_publisher(JointState, '/factr/joint_commands', qos_profile)

        # Subscriber to the follower Franka joint state topic published by the ROS2 controller
        self.franka_joint_state_sub = self.create_subscription(
            JointState,
            '/franka/joint_states',
            self._franka_joint_state_callback,
            qos_profile,
        )

        if self.enable_torque_feedback:
            # Optional subscriber for a dedicated external torque topic
            # Subscribe to /franka/joint_torques but only extract the 'effort' field
            self.franka_joint_torque_sub = self.create_subscription(
                JointState,
                '/franka/joint_torques',
                lambda msg: self._franka_joint_torque_callback(
                    type('EffortOnlyJointState', (), {'effort': list(msg.effort)})()
                ),
                qos_profile,
            )
            
    def _franka_joint_state_callback(self, msg: JointState):
        self.latest_franka_state = msg

    def _franka_joint_torque_callback(self, msg: JointState):
        self.latest_franka_torque = msg

    def _republish_franka_state(self):
        if self.latest_franka_state is None:
            return
        obs_msg = JointState()
        obs_msg.header.stamp = self.latest_franka_state.header.stamp
        obs_msg.header.frame_id = self.latest_franka_state.header.frame_id
        obs_msg.name = list(self.latest_franka_state.name)
        obs_msg.position = list(self.latest_franka_state.position)
        obs_msg.velocity = list(self.latest_franka_state.velocity)
        obs_msg.effort = list(self.latest_franka_state.effort)
        return obs_msg
        
    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque
    
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper
    
    def get_leader_arm_external_joint_torque(self):
        if self.latest_franka_state is None or len(self.latest_franka_state.effort) < self.num_arm_joints:
            if not self._missing_torque_warning_issued:
                self.get_logger().warn(
                    f"FACTR TELEOP {self.name}: No Franka torque data yet; force-feedback disabled until messages arrive."
                )
                self._missing_torque_warning_issued = True
            return np.zeros(self.num_arm_joints)

        external_torque = np.array(
            self.latest_franka_state.effort[:self.num_arm_joints], dtype=float
        )
        if hasattr(self, "obs_franka_torque_pub"):
            self.obs_franka_torque_pub.publish(create_array_msg(external_torque))
        self._missing_torque_warning_issued = False
        return external_torque

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        _ = leader_gripper_pos  # gripper commands are currently unused
        # send leader arm position as joint position target to the follower Franka arm
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = [f'franka_joint_{i+1}' for i in range(self.num_arm_joints)]
        cmd_msg.position = list(map(float, leader_arm_pos))
        self.factr_cmd_pub.publish(cmd_msg)

        # publish the current Franka follower arm joint state to ROS for behavior cloning/data logging
        self._republish_franka_state()
        

def main(args=None):
    rclpy.init(args=args)
    factr_teleop_franka_zmq = FACTRTeleopFrankaZMQ()

    try:
        while rclpy.ok():
            rclpy.spin(factr_teleop_franka_zmq)
    except KeyboardInterrupt:
        factr_teleop_franka_zmq.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_teleop_franka_zmq.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

