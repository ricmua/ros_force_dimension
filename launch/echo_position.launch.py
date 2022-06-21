""" Launch file for a Force Dimension ROS2 test.

A `launch file`_ for the ROS2 `launch system`_ that runs a the ROS2 command 
line utility to echo messages sent on the /robot/feedback/position topic. This 
can be useful for basic testing and debugging.

See the launch architecture_ documentation for further details.

.. _launch system: https://docs.ros.org/en/galactic/Tutorials/Launch-system.html
.. _launch file: https://docs.ros.org/en/galactic/Tutorials/Launch-Files
                 /Creating-Launch-Files.html
.. _architecture: https://github.com/ros2/launch/blob/master/launch/doc
                  /source/architecture.rst
"""

# Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
# 
# Created by: a. whit. (nml@whit.contact)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.


from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    
    server_node \
        = Node(package='force_dimension', executable='node')
    
    cmd = ['ros2', 'topic', 'echo', '/robot/feedback/position']
    echo_cmd = ExecuteProcess(cmd=cmd, output='screen', emulate_tty=True)
    delayed_echo_cmd = TimerAction(period=0.200, actions=[echo_cmd])
    
    return LaunchDescription([server_node, delayed_echo_cmd])
    
  

