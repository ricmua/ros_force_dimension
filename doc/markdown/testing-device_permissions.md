<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

### Linux device permissions

On Linux systems, testing of the Force Dimension SDK might fail with an error 
message that resembles the following:

```
user@host:/path/to/sdk$ bin/gravity 
Force Dimension - Gravity Compensation Example 3.13.2
Copyright (C) 2001-2021 Force Dimension
All Rights Reserved.

error: cannot open device (no device found)
```

This likely indicates that there is a udev / permissions issue. The issue is 
explained more by [Dan O'Shea](https://github.com/djoshea/haptic-control) 
(@djoshea on GitHub). The quickest way to resolve this is to just use elevated 
permissions when running the test: `sudo bin/gravity`. However, a preferable 
solution is to either use the udev rules file provided by O'Shea, the rules 
file included in this package, or to construct your own. The recommended 
complete solution is as follows:

1. Unplug the USB cable from the Force Dimension device.
2. In a terminal, run `udevadm monitor`.
3. Plug in the Force Dimension device, and then type CTRL-C in the terminal.
    * Information about the device should be printed in the terminal.
4. Copy the device path (or name) from the `udevadm` output.
    * This is likely prefixed with `/devices`, and displayed to the right of 
      an "add" command.
5. Run the udev info command.
    * `udevadm info --attribute-walk --path="/devices/path/from/udevadm/monitor"`
    * The path is taken from the previous step.
6. Copy the vendor and and product IDs from the output of this command. For the 
   Novint Falcon, they should be **0403** and **cb48**.
    * This can be verified with the command `lsusb -v -d 0403:cb48`.
    * The vendor and product ID should also be visible in the `dmesg` log.
7. Create the file `/etc/udev/rules.d/11-forcedimension.rules` and add the 
   following line:
    * `SUBSYSTEM=="usb", ACTION=="add", ATTR{idVendor}=="0403", ATTR{idProduct}=="cb48", MODE:="0666", GROUP:="force_dimension"`
    * The appropriate vendor and product IDs are inserted in the appropriate 
      place.
    * Save the file.
8. Issue the command `udevadm control --reload-rules`, and/or 
   `udevadm control --reload`.
9. In a terminal, create the new group `force_dimension`, and 
  [add the relevant user to the group].
10. Either reboot the computer, re-login to a new shell, or otherwise 
    [update the user group information].
11. Re-run the Force Dimension application (e.g., `bin/gravity`). Verify 
    error-free completition.

A sample `11-forcedimension.rules` file is included in the root of this 
project.

#### Links

* [udev - Linux dynamic device management](https://wiki.debian.org/udev)
* [An introduction to Udev: The Linux subsystem for managing device events](https://opensource.com/article/18/11/udev)
* [Writing udev rules](http://reactivated.net/writing_udev_rules.html)


<!------------------------------------------------------------------------------
  REFERENCES
------------------------------------------------------------------------------->

[add the relevant user to the group]: https://www.howtogeek.com/50787/add-a-user-to-a-group-or-second-group-on-linux/

[update the user group information: https://superuser.com/questions/272061/reload-a-linux-users-group-assignments-without-logging-out
