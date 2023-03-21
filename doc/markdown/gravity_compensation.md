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

> To prevent user fatigue and to increase accuracy during manipulation, Force 
  Dimension haptic devices features gravity compensation. When gravity 
  compensation is enabled, the weights of the arms and of the end-effector are 
  taken into account and a vertical force is dynamically applied to the 
  end-effector on top of the user command. Please note that gravity 
  compensation is computed on the host computer, and therefore only gets 
  applied whenever a force command is sent to the device by the application.

The Force Dimension ROS2 node allows for the configuration of gravity 
compensation via [ROS2 parameters][ros2_parameters].

## Parameters

Gravity compensation is controlled via two parameters: ``gravity_compensation`` 
and ``effector_mass_kg``. The ``gravity_compensation`` parameter is a Boolean 
value that simply determines whether or not gravity compensation is enabled. 
The ``effector_mass_kg`` defines the mass of the robotic end-effector, in 
kilograms. This value is used by the Force Dimension SDK to compute a 
compensatory force.[^effector_mass] The default value is reported by the Force 
Dimension node during startup configuration, and is recorded to the text 
log. The default value for the Novint Falcon is ``0.190``. The default value 
for the delta.3 is ``0.279``.

[^effector_mass]: The effector mass is set via the [dhdSetEffectorMass] 
                  function in the Force Dimension SDK. Gravity compensation is 
                  enabled via the [dhdSetGravityCompensation] function.

The following is a sample [YAML configuration file][ros2_yaml_config] for 
setting the gravity compensation parameters:

```
/robot/force_dimension:
  ros__parameters:
    gravity_compensation: true
    effector_mass_kg: 0.279
```

Both parameters trigger an update of the robot's configuration immediately, but 
the forces applied to the end effector do not change until a new command is 
issued. This is worth emphasizing: _After a change in parameters, the 
compensatory forces applied to the robotic end effector are updated only after 
the next force command is issued to the robot_.

Gravity compensation is most effective when the ``effector_mass_kg`` parameter 
is close to the true mass of the robotic endpoint effector. When the parameter 
setting is slightly less than the true value, the robot can be expected to 
descend at a slower pace than it would without compensation. When the parameter 
setting is slightly higher than the true value, the effector will rise. 
Identifying the optimal parameter value may require some trial-and-error
[calibration](#testing-and-calibration), if the mass of the effector is 
uncertain.

## Testing and calibration

In a [configured ROS2 environment][configure_ros2_environment], run the Force 
Dimension node:

```
ros2 run force_dimension node
```

During initialization, the node will report the configured effector mass. For 
this example, a mass of ``0.190`` kg will be assumed.

In a second configured environment, simulate reduced-gravity by increasing the 
effector mass by 25% (``0.190 * 1.25 = 0.2375``):

```
ros2 param set /robot/force_dimension effector_mass_kg 0.2375
```

In the first ROS2 environment, the Force Dimension should report this parameter 
change to the log. However, the parameter change has no effect until a force 
command is delivered to the robot. To trigger the gravity compensation update, 
send a force command:

```
ros2 topic pub --once /robot/command/force geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: 0.0}"
```

At this point, the robot can be considered to be operating within a simulated 
physical environment, in which gravity has been reduced by 25%. Depending on 
the robot, and how it is set up, this might cause the end effector to rise 
toward the top of the workspace. If this does not happen, then increase the 
effector mass parameter by small increments -- re-issuing the force command for 
each parameter change -- until the end effector is observed to move upward.

To confirm the effect, disable gravity compensation:

```
ros2 param set /robot/force_dimension gravity_compensation false
```

For this parameter change to take effect, a force command must be issued to the 
robot, as before. _Be prepared to catch the end effector when the change takes 
effect_. With gravity compensation disabled, the force of gravity will 
typically cause the robotic effector to drop.




[dhdSetEffectorMass]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a21b58f37e0bd783f4744a8874ae7a02d

[fd_gravity_compensation]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhd_glossary.html#dhd_gravity

[dhdSetGravityCompensation]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a15dcf0e3c33142b2ac79fd023d03c641

[configure_ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

[ros2_parameters]: https://docs.ros.org/en/humble/Tutorials/Parameters/Understanding-ROS2-Parameters.html

[ros2_yaml_config]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html#ros2-param-dump

