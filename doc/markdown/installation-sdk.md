<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Force Dimension SDK installation

The software development kit (SDK) for Force Dimension haptic robotics must be 
installed for this package to build and function properly. The steps for 
installing the SDK are as follows:

1. Download the SDK from the [Force Dimension website].[^1] 
2. Extract the SDK to a desired location in the target computer's filesystem 
   (e.g., `/path/to/sdk` or `C:\path\to\sdk`).
3. The build system must then be made aware of the SDK via 
   [CMake configuration]. Copy the file 
   [`ForceDimensionSDKConfig.cmake`](/ForceDimensionSDKConfig.cmake) from the 
   root of this project into the root directory of the Force Dimension SDK 
   (e.g., `/path/to/sdk` or `C:\path\to\sdk`). Then set the environmental 
   variable `ForceDimensionSDK_DIR` to point to the directory containing the 
   CMake config file.[^2] For example, issue a terminal command like the 
   following:
   
   * Linux: `export ForceDimensionSDK_DIR=/path/to/sdk`
   * Windows: `set ForceDimensionSDK_DIR=C:\path\to\sdk`
   
4. Set up the build environment. If building on Linux, then some 
   [additional configuration](/doc/markdown/installation-sdk-linux.md) is 
   required. If building on Windows, then an appropriate 
   [command line environment][msvc_terminal] must be initialized.

[^1]: At the time of writing this document, the current version is 3.16.1.

[^2]: This environmental variable does not need to be set persistently. It is 
      only required during the build process.

## Building the SDK

It is **not necessary** to build the examples provided with the Force Dimension 
SDK. The SDK can be used to build the ROS2 package as it is. However, it can be 
useful for confirming the installation, and for debugging with a native build.
For Linux sytems, the SDK build is accomplished by running the `make` command 
in a terminal at the root of the Force Dimension SDK path. 
For Windows systems, use MS Visual Studio to [build][msvc_build] the project. 
Remember to properly [initialize the environment][msvc_terminal].

## Testing the SDK

Verify the proper installation of the SDK by running one of the included 
binaries (e.g., `bin/gravity` or `bin\gravity.exe`).

Note: Force Dimension robots must be **calibrated** to function properly. See 
the section on [robot calibration](calibration.md) for further details.


<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[Force Dimension website]: https://www.forcedimension.com/software/sdk

[CMake configuration]: https://cmake.org/cmake/help/latest/guide/using-dependencies/index.html

[msvc_terminal]: https://docs.microsoft.com/en-us/cpp/build/building-on-the-command-line?view=msvc-170


[msvc_build]: https://docs.microsoft.com/en-us/visualstudio/ide/compiling-and-building-in-visual-studio?view=vs-2022

