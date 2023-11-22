<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

# Force Dimension SDK installation (Linux)

Some additional steps might be required in order to build and run the Force 
Dimension SDK on Linux.

## Dependencies

The Force Dimension SDK examples and the ROS2 node have some external 
dependencies. In particular, the build process expects access to the 
`libusb-1.0` library. Using the Ubuntu package manager, this can be installed 
by invoking the command `sudo apt install libusb-1.0-0-dev`.

The following Ubuntu package manager invocations might also be required, in 
order to build the examples provided with the Force Dimension SDK.

* `sudo apt install libxrender-dev`
* `sudo apt install libxcursor-dev`
* `sudo apt install libxrandr-dev`
* `sudo apt install libxinerama-dev`
* `sudo apt install libglfw3-dev`

## Libraries

Packaged with the SDK, Force Dimension provides both static and shared 
libraries (with .a and .so extensions, respectively).

### Static

The static libraries are automatically located and added to the project via 
[`ForceDimensionSDKConfig.cmake`](/ForceDimensionSDKConfig.cmake). However, 
second order dependencies are not.[^1] For this reason, it is necessary to add 
the dependencies on the command line at the time of build. As an example, the 
following command is the minimal requirement for a successful build.

```bash
> colcon build --cmake-args -DCMAKE_CXX_STANDARD_LIBRARIES="-lusb-1.0"
```

[^1]: In a future release, the dependencies will be added to the project files 
      -- likely as part of 
      [`ForceDimensionSDKConfig.cmake`](/ForceDimensionSDKConfig.cmake) (see 
      the ament documentation about [finding dependencies]).


### Shared

As it is packaged for download, the SDK contains shared objects for the DHD and 
DRD APIs. These are stored at paths that resemble 
`lib/release/lin-x86_64-gcc/libdhd.so.3.16.1`. As explained by the 
[Program Library HOWTO] by Wheeler:

> Every shared library has a special name called the "[soname]". The soname has 
  the prefix `lib`, the name of the library, the phrase `.so`, followed by a 
  period and a version number that is incremented whenever the interface 
  changes... A fully-qualified soname includes as a prefix the directory it's 
  in; on a working system a fully-qualified soname is simply a symbolic link to 
  the shared library's "real name".

> Every shared library also has a "real name", which is the filename containing 
  the actual library code. The real name adds to the soname a period, a minor 
  number, another period, and the release number... The minor number and 
  release number support configuration control by letting you know exactly what 
  version(s) of the library are installed...
  
> In addition, there's the name that the compiler uses when requesting a 
  library, (I'll call it the "linker name"), which is simply the soname 
  without any version number.

It is necessary to inform the build-time and run-time systems where to find 
these libraries.

> When you install a new version of a library, you install it in one of a few 
  special directories and then run the program 
  [ldconfig]([8][ldconfig man page]). ldconfig examines the existing files and 
  creates the sonames as symbolic links to the real names, as well as setting 
  up the cache file /etc/ld.so.cache (described in a moment).

In order to install the Force Dimension SDK, therefore, `ldconfig` must be 
invoked.[^2] For example, assuming the path from above:

<!--
```bash
sudo ldconfig path/to/sdk/lib/*/*
```
-->

```bash
> sudo ldconfig path/to/sdk/lib/release/lin-x86_64-gcc
```

[^2]: Rather than specifying a path to `ldconfig`, it might be easier to simply 
      transfer the shared objects to expected filesystem library paths (e.g., 
      `/usr/lib`) before running `ldconfig`.

This adds the Force Dimension libraries to the linker cache, and creates 
soname references. The latter manifest as symbolic links -- created at the 
specified path -- that point to the libraries' real names. 

However, this is not quite enough.

> ldconfig doesn't set up the linker names; typically this is done during 
  library installation, and the linker name is simply created as a symbolic 
  link to the "latest" soname... 

Therefore, two more symbolic links must be manually created. These links point 
to the symbolic links created by `ldconfig`. Again assuming the filesystem path 
from the example above, the commands are:

```bash
> cd path/to/sdk/lib/release/lin-x86_64-gcc
> ln -s libdhd.so.3 libdhd.so
> ln -s libdrd.so.3 libdrd.so
```

To force the use of shared libraries, `colcon` can be invoked with the CMake 
flag [`BUILD_SHARED_LIBS`] set.

```bash
> colcon build --cmake-args -DBUILD_SHARED_LIBS=ON
```

<!--
Note that the shared objects must be available at runtime. This might require 
another invocation of the `ldconfig` command.
-->


## Device permissions

If the SDK builds, but the applications fail during testing, then 
[device permissions](testing-device_permissions.md) might need to be adjusted.



<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[soname]: https://en.wikipedia.org/wiki/Soname

[ldconfig]: https://man7.org/conf/lca2006/shared_libraries/slide8a.html

[ldconfig man page]: https://man7.org/linux/man-pages/man8/ldconfig.8.html

[Program Library HOWTO]: https://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html

[`BUILD_SHARED_LIBS`]: https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html#variable:BUILD_SHARED_LIBS





[shared libraries]: https://www.tecmint.com/understanding-shared-libraries-in-linux/


[source your ROS2 environment]: https://docs.ros.org/en/iron/Tutorials/Configuring-ROS2-Environment.html

[build_a_ros2_package]: https://docs.ros.org/en/iron/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package

[ROS2 build system]: https://docs.ros.org/en/iron/Concepts/About-Build-System.html

[Force Dimension website]: https://www.forcedimension.com/software/sdk

[CMake configuration]: https://cmake.org/cmake/help/latest/guide/using-dependencies/index.html


[finding dependencies]: https://docs.ros.org/en/iron/How-To-Guides/Ament-CMake-Documentation.html#finding-dependencies


