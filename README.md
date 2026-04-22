# boxes-brain
ROS2 software for a robotic character named "Boxes" as part of a theatrical production

install steps taken so far:

wireless xbox controller dongle
- install xone driver (look for it on gh)
- make sure to also install the required dependencies which they don't call out (cabextract and DKMS since we're on a lite version of the linux distro
- `sudo apt install joystick` - this installs useful tools for testing out the interface such as `evtest` and `fftest`

ros distro
- use ros2 jazzy
- had to reflash pi's os with ubuntu 24.04 because I didn't want to have to deal with docker containers
- followed (https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html#ubuntu-linux-on-raspberry-pi-with-binary-ros-2-install)[these instructions]
- also (https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html#ubuntu-linux-on-raspberry-pi-with-binary-ros-2-install)[these]

todo:
- how to enable uart on gpio pins on a pi5 running. 24.04 headless noble
