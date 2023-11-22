
<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Example Python client

A [Python client example](/scripts/run_example) is included with this package, 
in order to illustrate how a client might interact with the Force Dimension 
node. The client is designed to be as simple as possible: it just prints the 
current 3D position of the robot end-effector to the ROS log, each time it 
receives a message on the position [topic](doc/markdown/topics.md) 
(`feedback/position`, by default).

In order to make it easier to run the example, a 
[launch file](/launch/example.launch.py) has been provided. Once the ROS2 
workspace has been built -- and the ROS2 environment sourced -- the example can 
be started with the following terminal command:

```bash
> ros2 launch force_dimension example.launch.py
```

If the robot is properly configured and calibrated, then this should output 
periodic position measurements to the terminal. The output should resemble the 
following (log prefixes have been removed for clarity):
 
```
All log files can be found below /path/to/logs
Default logging verbosity is set to INFO
process started with pid [123456]
process started with pid [123457]
[robot.force_dimension]: Initializing the Force Dimension interface.
[robot.force_dimension]: Force Dimension device detected: Falcon
[robot.force_dimension]: Force Dimension interface initialized.
[robot.force_dimension]: Failure getting communication refresh rate: no error
[robot.force_dimension]: Sample timer initialized: Interval (ms) = 25.000000
[robot.force_dimension]: BASELINE EFFECTOR MASS: 0.190000 kg
[robot.force_dimension]: Effector mass set to 0.190000 kg.
[robot.force_dimension]: Gravity compensation: Enabled
[robot.example_client]: position=(x: -0.00513, y: -0.00623, z: -0.058)
[robot.example_client]: position=(x: -0.00552, y: -0.00582, z: -0.0581)
[robot.example_client]: position=(x: -0.0103, y: -0.0429, z: -0.0417)
[robot.example_client]: position=(x: 0.000884, y: -0.0253, z: -0.0399)
[robot.example_client]: position=(x: -0.0242, y: -0.0266, z: -0.00622)
[robot.example_client]: position=(x: -0.00687, y: -0.0608, z: -0.00351)
[robot.example_client]: position=(x: 0.00169, y: -0.0494, z: -0.046)
[robot.example_client]: position=(x: 0.00899, y: -0.0187, z: -0.0198)
[robot.example_client]: position=(x: 0.00442, y: -0.0198, z: 0.0125)
[robot.example_client]: position=(x: -0.0272, y: -0.0291, z: 0.0606)
[robot.example_client]: position=(x: -0.0279, y: -0.0291, z: 0.0604)
[robot.example_client]: position=(x: -0.0294, y: -0.0282, z: 0.0615)
[robot.example_client]: position=(x: -0.0294, y: -0.0281, z: 0.0615)
[robot.example_client]: position=(x: -0.0294, y: -0.0281, z: 0.0615)
[robot.example_client]: position=(x: -0.0294, y: -0.0281, z: 0.0615)
[robot.example_client]: position=(x: -0.0294, y: -0.0281, z: 0.0616)
...
```

Moving the robot endpoint should change the reported positions. Terminate the 
example by striking `CTRL-C`.

