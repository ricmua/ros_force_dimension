---
author: a.whit (nml@whit.contact)
date: May 2022
---

<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Gravity Compensation

The [Force Dimension SDK](force_dimension.md) documentation describes 
[gravity compensation][fd_gravity_compensation] as follows:

> To prevent user fatigue and to increase accuracy during manipulation, Force Dimension haptic devices features gravity compensation. When gravity compensation is enabled, the weights of the arms and of the end-effector are taken into account and a vertical force is dynamically applied to the end-effector on top of the user command. Please note that gravity compensation is computed on the host computer, and therefore only gets applied whenever a force command is sent to the device by the application.

The Force Dimension ROS2 node allows for the configuration of gravity compensation via ROS2 parameters.

## Parameters

Gravity compensation is controlled via two parameters: ``gravity_compensation`` 
and ``effector_mass_kg``. The ``gravity_compensation`` parameter is a Boolean 
value that simply determines whether or not gravity compensation is enabled. 
The ``effector_mass_kg`` defines the mass of the robotic end-effector, in 
kilograms. This value is used by the Force Dimension SDK to compute a 
compensatory force.[^effector_mass] The default value is reported by the Force 
Dimension node during startup configuration, and will be recorded in the text 
log. The default value for the Novint Falcon is ``0.190``. The default value 
for the delta.3 is ``0.279``.

[^effector_mass]: The effector mass is set via the [dhdSetEffectorMass] function in the Force Dimension SDK. Gravity compensation is enabled via the 
[dhdSetGravityCompensation] function.

The following is a sample YAML configuration file for setting the gravity compensation parameters:

```
/robot/force_dimension:
  ros__parameters:
    gravity_compensation: true
    effector_mass_kg: 0.279
```

Both parameters are currently configured to trigger an update whenever they are 
modified. So -- for example -- changing the ``gravity_compensation`` parameter 
to ``false`` while the node is running should be expected to instantly disable 
gravity compensation.[^Technically, the update will take effect the instant 
that a new force command is delivered to the robot.]

## Testing and calibration

To run the Force Dimension Node:

1. Navigate to the relevant ROS2 workspace (i.e., either the _testing_ or _stable_ branch of the NML NHP workspace):
   ```cd D:\dev\nml_nhp\ros_workspace-testing```
2. Prepare a ROS2 environment by [sourcing the ROS2 setup script](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-ros-2-environment): 
   
   ```
   D:\dev\ROS2\Galactic\setup.bat
   ```
3. Prepare the overlay environment by [sourcing the local workspace setup script](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay): ``install\local_setup.bat``.
4. Start the ROS2 node: ``ros2 run force_dimension node``

To dynamically adjust parameters:

1. Navigate to the relevant ROS2 workspace (i.e., either the _testing_ or _stable_ branch of the NML NHP workspace): ``cd D:\dev\nml_nhp\ros_workspace-testing``.
2. Prepare a ROS2 environment by [sourcing the ROS2 setup script](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-ros-2-environment):
   
   ```D:\dev\ROS2\Galactic\setup.bat```
3. Enable gravity compensation:
   
   ```
   ros2 param set /robot/force_dimension gravity_compensation true
   ```
4. Specify effector mass (kilograms):
   
   ```
   ros2 param set /robot/force_dimension effector_mass_kg 0.45
   ```
5. Gravity compensation updates only take effect after a force command has been 
   delivered to the robot. Trigger an update by sending a single force command: 
   
   ```
   ros2 topic pub --once \
      /robot/force geometry_msgs/msg/Vector3 \
      "{x: 0.0, y: 0.0, z: 0.0}"
   ```
6. In order to calibrate gravity compensation, try a second effector mass 
   setting:
   
   ```
   ros2 param set /robot/force_dimension effector_mass_kg 0.55
   ```
7. Trigger an update:
   
   ```
   ros2 topic pub --once \
      /robot/force geometry_msgs/msg/Vector3 \
      "{x: 0.0, y: 0.0, z: 0.0}"
   ```
8. If the effector mass is not known _a priori_, then repeat the prior two 
   steps until you've found an effector mass that produces the desired result.

Gravity compensation is most effective when the ``effector_mass_kg`` parameter 
is close to the true mass of the robotic endpoint effector. When the parameter 
setting is slightly less than the true value, the robot can be expected to 
descend at a slower pace than it would without compensation. When the parameter 
setting is slightly higher than the true value, the effector will rise. 
Identifying the optimal parameter value may require some trial-and-error 
guessing.



[dhdSetEffectorMass]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a21b58f37e0bd783f4744a8874ae7a02d

[fd_gravity_compensation]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhd_glossary.html#dhd_gravity

[dhdSetGravityCompensation]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a15dcf0e3c33142b2ac79fd023d03c641


