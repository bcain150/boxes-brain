# boxes-brain
ROS2 software for a robotic character named "Boxes" as part of a theatrical production

install steps taken so far:

wireless xbox controller dongle
- install xone driver (look for it on gh)
- make sure to also install the required dependencies which they don't call out (cabextract and DKMS since we're on a lite version of the linux distro
- `sudo apt install joystick` - this installs useful tools for testing out the interface such as `evtest` and `fftest`

NOTE: I experienced in the next step that it seemed to unregister xone from dkms for some reason... 
- I had to reregister xone with dkms and then reinstall specific linux headers, after doing so I verified that UART works still. Secondly install the meta-package headers and it should auto update and not break again
 - `sudo apt install linux-headers-6.8.0-1052-raspi`
 - meta headers: `sudo apt install linux-headers-raspi` _this auto rebuilt xone and reinstalled without dkms_

enabling uart on GPIO pins 14 & 15 (this is for raspberry pi5)
1. edit config.txt in `/boot/firmware/config.txt`
    - add the following lines:
        - `enable_uart=1`
        - `dtoverlay=uart0-pi5`
        - `dtoverlay=disable-bt` _not sure how crucial this is_
2. disable the debug console for ubuntu from constantly writing to /dev/tty/AMA0
    - remove the following text from `/boot/firmware/cmdline.txt` (make sure it stays on one line, no extra spaces or `,`):
        - `console=serial0,115200`
3. stop the serial getty service which allows people to login from uart
    - `sudo systemctl stop serial-getty@ttyAMA0.service`
    - `sudo systemctl mask serial-getty@ttyAMA0.service`
4. reboot
    - `sudo reboot`


ros distro
- use ros2 jazzy
- had to reflash pi's os with ubuntu 24.04 because I didn't want to have to deal with docker containers
- followed (https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html#ubuntu-linux-on-raspberry-pi-with-binary-ros-2-install)[these instructions]
- also (https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html#ubuntu-linux-on-raspberry-pi-with-binary-ros-2-install)[these]

# INFO ON SETTING UP THE REPO TO RUN LAUNCH ON STARTUP

## Boot-on-startup setup

### Install

```bash
sudo cp scripts/boxes-brain-launch.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/boxes-brain-launch.sh
sudo cp systemd/boxes-brain.service /etc/systemd/system/
sudo cp systemd/journald@boxes.conf /etc/systemd/
```

### Enable

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now boxes-brain.service
```

### Verify

```bash
sudo systemctl status boxes-brain.service
journalctl --namespace=boxes -u boxes-brain.service -f
```
