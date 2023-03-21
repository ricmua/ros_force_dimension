<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

The Robot Operating System ([ROS][ros_wikipedia]) is a software framework 
(software development kit and middleware) that aims to simplify the development 
and distribution of robotics software and control systems. More generally, ROS

> provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management. 

Among other functionality, ROS provides the "plumbing" for linking diverse 
software and hardware components. At its core, it is a form of 
[message-oriented middleware][mom_wikipedia] (MOM), which

> is software or hardware infrastructure supporting sending and receiving messages between distributed systems. MOM allows application modules to be distributed over heterogeneous platforms and reduces the complexity of developing applications that span multiple operating systems and network protocols. 

In addition to the [advantages][mom_advantages] of a MOM architecture, the ROS2 [ecosystem][ros_ecosystem] also provides tools, capabilities, and support that 
simplifies the process of developing solutions involving distributed sensor and 
robotics systems. These capabilities help developers to 
[avoid "reinventing the wheel"][why_ros].

This package provides an interface between ROS and the Force Dimension SDK, so 
that applications involving Force Dimension haptic devices can leverage the 
many advantages of ROS. It is our hope that this will accelerate the process of 
developing haptics applications.

[ros_wikipedia]: https://en.wikipedia.org/wiki/Robot_Operating_System
[why_ros]: https://www.ros.org/blog/why-ros/
[ros_ecosystem]: https://www.ros.org/blog/ecosystem/
[mom_wikipedia]: https://en.wikipedia.org/wiki/Message-oriented_middleware
[mom_advantages]: https://en.wikipedia.org/wiki/Message-oriented_middleware#Advantages
[force_dimension_sdk]: https://www.forcedimension.com/software/sdk
