### Linux ``wsl-usb`` 

WSL provides support with connecting USB devices to Linux distributions running WSL 2 using the USB/IP open-source project, [usbipd-win](https://github.com/dorssel/usbipd-win). 

### How to install

Run the installer (.msi) from the [latest release](https://github.com/dorssel/usbipd-win/releases) on the Windows machine where your USB device is connected.

User space tools for Linux are also needed. Open a WSL terminal and install the support packages.
```bash
sudo apt install linux-tools-generic hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
```

### Connecting a device

Using a Powersheel terminal as administrator enter the following command:
```bash
usbipd wsl list - Note the BUS ID value for the USB device you want to attach to WSL
```

For example, an output for a Sigma7 robot will include the following:
```
BUSID  VID:PID    DEVICE
1-6    1451:0403  Force Dimension sigma.7
```

To attach a new device to WSL, use the attach command (with the bus-id from above as an example):

```bash
usbipd wsl attach --busid 1-6
```

Verify that the device will show up in your Linux environment by typing into a WSL terminal `lsusb`. It should now list your attached device.

To detach a device from WSL, use the detach command:

```bash
usbipd wsl detach --busid 1-6
```

Note that by default devices are not shared with USBIP clients. You will need to re-attach your devices if starting a new WSL session. To avoid using terminals, a GUI application also is available that handles attaching/detaching devices between Windows and WSL Linux. The latest installer is always available [on their releases page](https://gitlab.com/alelec/wsl-usb-gui/-/releases). Installation of dependencies should be handled automatically at the end of the MSI installer.

#### Links

* [usbipd-win](https://github.com/dorssel/usbipd-win)
* [WSL Connect USB](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
* [wsl_usb_gui](https://github.com/featherbear/wsl-usb-gui)


