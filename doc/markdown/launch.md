<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

The ROS2 [launch system] is used here to provide simple commands that 
simultaneously start [multiple nodes][ros2_multiple_launch], in order to 
facilitate testing, debugging, and demonstrations involving the Force Dimension 
pacakge.

## Quickstart

Use the ROS2 command line to launch these scripts. For example:

```ros2 launch force_dimension echo_position.launch.py```

## Overview

The ROS2 launch system has been [described][ros2_design_launch] as follows:

> The launch system in ROS is responsible for helping the user describe the 
  configuration of their system and then execute it as described. The 
  configuration of the system includes what programs to run, where to run them, 
  what arguments to pass them, and ROS specific conventions which make it easy 
  to reuse components throughout the system by giving them each different 
  configurations.

The launch system is described in greater detail in the 
[architecture documentation][ros2_launch_architecture].


## Launch files

The following launch files are available as part of the Force Dimension 
package:

* ``echo_position.launch.py``: This launch command will start the Force 
  Dimension node and the ros2 topic [echo][ros2_topic_echo] command line 
  [utility][ros2_command_line]. While running, the position of the robotic 
  end-effector will be displayed as scrolling text on the screen.
* ``record_bag.launch.py``: This launch command will start the Force 
  Dimension node and the ros2 bag utility for 
  [recording data][ros2_record_data]. The utility will record Force Dimension 
  data with the default settings.



[launch system]: https://docs.ros.org/en/humble/Tutorials/Launch/CLI-Intro.html

[ros2_design_launch]: https://design.ros2.org/articles/roslaunch.html

[ros2_launch_architecture]: https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst

[ros2_multiple_launch]: https://docs.ros.org/en/humble/Tutorials/Launch/Launch-system.html

[ros2_launch_files]: https://docs.ros.org/en/humble/Tutorials/Launch/Creating-Launch-Files.html

[ros2_command_line]: https://docs.ros.org/en/humble/Concepts/About-Command-Line-Tools.html

[ros2_topic_echo]: https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html#ros2-topic-echo

[ros2_record_data]: https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html


