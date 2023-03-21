
<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->


# Robot calibration

Force Dimension robots require periodic calibration. Until the robot is 
calibrated, it will not function. From the [sigma.7 manual][sigma_7_manual_20]:

> Calibration of the haptic device controller is necessary to obtain accurate, 
  reproducible localization of the end-effector within the workspace of the 
  haptic device... the procedure only needs to be performed once each time the 
  device is powered ON.

[sigma_7_manual_20]: http://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/user%20manual%20-%20sigma.7.pdf#page=20

### Novint Falcon

For the Novint Falcon, the need to calibrate is indicated by a red status 
light, on the forward face of the robot. Try gently moving the robot endpoint 
around the limits of the workspace. Or, as it is put in the 
[delta.x manual][delta_x_manual_17], the robot can be calibrated:

> by slowly sweeping each axis from end-stop to end-stop.

After a few seconds of motion, the robot should spring to life.

[delta_x_manual_17]: http://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/user%20manual%20-%20delta.x.pdf#page=17

### delta.x

For the delta.x, the status LED on the [control unit][delta_x_manual_16] will 
blink when calibration is required. According to the 
[delta.x manual][delta_x_manual_17], calibration can be accomplished in two 
ways:

> 1. by slowly sweeping each axis from end-stop to end-stop. This is the 
  recommended calibration method.

> 2. by holding the force button on the control unit for 2 seconds while 
  maintaining the end-effector at the calibration position (axis 0 retracted, 
  axis 1 and 2 extended, see figure 10). Make sure all axes are resting on 
  their respective mechanical

[delta_x_manual_16]: http://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/user%20manual%20-%20delta.x.pdf#page=16

### sigma.7

For the sigma.7 robot, calibration is **automatic**. According to the 
[sigma.7 manual][sigma_7_manual_20]:

> The automatic calibration procedure is performed by software using the Force 
  Dimension SDK, for example by launching the application "HapticInit" which 
  automatically drives the device throughout its workspace. Please do not touch 
  the device during this automatic calibration procedure.

The ``HapticInit.exe`` application is contained within the ``bin`` directory of the Force Dimension SDK for Windows.


