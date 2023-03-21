<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

Some Force Dimension robots are equipped with a gripper. The 
[topics documentation](topics.md#feedback-topics) describes how 
the Force Dimension node communicates information about the state of the 
gripper. Also see the parameters documentation pertaining to 
[sample decimation](parameters.md#feedback-sample-decimation).

A gripper can be used to emulate buttons. See the parameters documentation 
section pertaining to 
[button emulation](parameters.md#button-emulation) for further 
information.

## TODO

Interfaces to a number of gripper-related functions provided by the Force 
Dimension SDK have not yet been implemented:

* Gripper position feedback: [dhdGetGripperThumbPos] and 
  [dhdGetGripperFingerPos];
* Gripper force feedback: [dhdGetForceAndTorqueAndGripperForce];
* Gripper velocity feedback: [dhdGetGripperAngularVelocityRad];
* Gripper active force commands: [dhdSetForceAndGripperForce].

[dhdGetGripperThumbPos]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#af3df18474e589b3335776f271921c0fd 

[dhdGetGripperFingerPos]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a46954d4c7743f356d2ae2327d0c22f2e

[dhdSetForceAndGripperForce]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#aa2beb27a94c693149603619d44fc2725

[dhdGetGripperAngularVelocityRad]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#aceda935c42317193308c351608a3ae40

[dhdGetForceAndTorqueAndGripperForce]: https://downloads.forcedimension.com/sdk/doc/fdsdk-3.14.0/dhd/dhdc_8h.html#a93c14156759e73c48370cae71a11de46


