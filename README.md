# boxes-brain
ROS2 software for a robotic character named "Boxes" as part of a theatrical production

install steps taken so far:

wireless xbox controller dongle
- install xone driver (look for it on gh)
- make sure to also install the required dependencies which they don't call out (cabextract and DKMS since we're on a lite version of the linux distro
- `sudo apt install joystick` - this installs useful tools for testing out the interface such as `evtest` and `fftest`
