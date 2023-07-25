<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Installation

Installation of this ROS2 package has been tested with Ubuntu 20.04 and Windows 
10.

## Force Dimension SDK

In order to build the Force Dimension ROS2 node, the Force Dimension SDK must 
be available to the [ROS2 build system][ros2_build_system]. The SDK also 
provides all drivers and libraries that are essential for interfacting with a 
Force Dimension robot.

To install the Force Dimension SDK, download it from the 
[Force Dimension website][force_dimension_sdk].[^1] Extract the SDK to a 
desired location in the target computer's filesystem (e.g., ``/path/to/sdk`` or 
``C:\path\to\sdk``). The build system must then be made aware of the SDK via 
[CMake configuration][cmake_using_dependencies]. Copy the file
``ForceDimensionSDKConfig.cmake`` from the root of this project into the root 
directory of the Force Dimension SDK (e.g., ``/path/to/sdk`` or 
``C:\path\to\sdk``). Set the environmental variable ``ForceDimensionSDK_DIR`` 
to point to the directory containing the CMake config file.[^2] For example, 
issue a terminal command like the following:

* Linux: ``export ForceDimensionSDK_DIR=/path/to/sdk``
* Windows: ``set ForceDimensionSDK_DIR=C:\path\to\sdk``

[^1]: At the time of writing this document, the current version is 3.13.

[^2]: This environmental variable does not need to be set persistently. It is only required during the build process.

[ros2_build_system]: https://docs.ros.org/en/humble/Concepts/About-Build-System.html

[force_dimension_sdk]: https://www.forcedimension.com/software/sdk

[cmake_using_dependencies]: https://cmake.org/cmake/help/latest/guide/using-dependencies/index.html

### Additional steps: Linux

Some additional steps might be required for Linux builds. In particular, some 
additional [dependencies](installation-linux_dependencies.md) 
might need to be installed via a package manager, and dynamic libraries might 
need to be registered with the build system via 
[ldconfig](installation-ldconfig.md).

If the SDK builds, but the applications fail during testing, then 
[device permissions](testing-device_permissions.md) might need to 
be adjusted.

### Additional steps: Windows

If building the Force Dimension SDK from the command line, then an appropriate 
[command line environment][msvc_terminal] must be initialized.

[msvc_terminal]: https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line?view=msvc-170

### Build

The SDK build is accomplished in the following way:

* Linux: Run the ``make`` command in a terminal at the root of the Force 
  Dimension SDK path.
* Windows: Use MS Visual Studop to [build][msvc_build] the project.

[msvc_build]: https://docs.microsoft.com/en-us/visualstudio/ide/compiling-and-building-in-visual-studio?view=vs-2022

### Test

Once the SDK is built, test it by running one of the included binaries (e.g., 
``bin/gravity`` or ``bin\gravity.exe``). 

Note: Force Dimension robots must be **calibrated** to function properly. See 
the section on [robot calibration](calibration.md) for further 
details.



## ROS2 package

The Force Dimension ROS2 package is built in the 
[standard fashion][build_a_ros2_package],[^3] using ``colcon``.[^4] If the 
Force Dimension SDK has been installed correctly, then no additional steps are 
required.

[^3]: Do not forget to configure or 
      [source your ROS2 environment][configure_ros2_environment].

[^4]: Reminder: The ``ForceDimensionSDK_DIR`` environmental variable must be 
      set during the build process. See the instructions for installing the 
      Force Dimension SDK.


[configure_ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[build_a_ros2_package]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package

