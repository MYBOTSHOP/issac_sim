#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of MYBOTSHOP GmbH nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

import time
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class IsaacController(Node):

    def __init__(self):
        super().__init__('h1_isaac_controller')
        self.get_logger().info(f'{self.colorize("Activating Isaac Controller Node!","yellow")}')

        # Parameters Declaration
        self.declare_parameters(namespace='', parameters=[
            ('robot_topic', 'cookie'),
        ])
        self.param_robot_topic = self.get_parameter('robot_topic').value
        self.get_logger().info(f'{self.colorize(f"robot_topic: {self.param_robot_topic}","blue")}')

        # ROS Publishers
        self.pub_1 = self.create_publisher(JointState, 'joint_command', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.start_time = self.get_clock().now().to_msg().sec

    def timer_callback(self):
        msg = JointState()

        # Define joint names
        self.joint_names = [
            'left_hip_yaw_joint', 'right_hip_yaw_joint', 'torso_joint', 'left_hip_roll_joint',
            'right_hip_roll_joint', 'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint',
            'left_hip_pitch_joint', 'right_hip_pitch_joint', 'left_shoulder_roll_joint',
            'right_shoulder_roll_joint', 'left_knee_joint', 'right_knee_joint',
            'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint', 'left_ankle_joint',
            'right_ankle_joint', 'left_elbow_joint', 'right_elbow_joint'
        ]

        # Initialize joint positions, velocities, and efforts
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        current_time = self.get_clock().now().to_msg().sec
        time_elapsed = current_time - self.start_time

        # Update positions with a walking pattern
        amplitude_hip = 0.2  # Increased amplitude of hip oscillation
        amplitude_knee = 0.3  # Increased amplitude of knee oscillation
        frequency = 0.5  # Frequency of oscillation (Hz)

        # Calculate positions for the hips and knees to simulate walking
        self.joint_positions[7] = amplitude_hip * math.sin(2 * math.pi * frequency * time_elapsed) * 1e15 # left_hip_pitch_joint
        self.joint_positions[8] = amplitude_hip * math.sin(2 * math.pi * frequency * time_elapsed + math.pi)* 1e15  # right_hip_pitch_joint
        self.joint_positions[11] = amplitude_knee * math.sin(2 * math.pi * frequency * time_elapsed) * 1e15 # left_knee_joint
        self.joint_positions[12] = amplitude_knee * math.sin(2 * math.pi * frequency * time_elapsed + math.pi)  * 1e15# right_knee_joint 

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        # Log the joint positions for debugging
        self.get_logger().info(f"Publishing joint positions: {self.joint_positions}")

        self.pub_1.publish(msg)

    def colorize(self, text, color):
        color_codes = {
            'green': '\033[92m',
            'yellow': '\033[93m',
            'orange': '\033[38;5;208m',
            'blue': '\033[94m',
            'red': '\033[91m'
        }
        return color_codes[color] + text + '\033[0m'

    def destroy_node(self):
        self.get_logger().info(f'{self.colorize("Shutting down Isaac controller node","red")}')
        super().destroy_node()