# VideoRay ROS Packages

[![Build Status](https://travis-ci.org/RobustFieldAutonomyLab/videoray.svg?branch=master)](https://travis-ci.org/RobustFieldAutonomyLab/videoray)

## Getting Started

This package supports ROS Indigo, Jade and Kinetic. First, clone this repository and build the videoray package.

```
mkdir -p YOUR_WS/src && cd YOUR_WS/src
git clone https://github.com/RobustFieldAutonomyLab/videoray.git
catkin_init_workspace
cd .. && rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.sh
```

To avoid manually configuring device names every time, it's recommended to put [persistent naming rules](./99-usb-serial.rules) on the machine.

```
sudo mv ./99-usb-serial.rules /etc/udev/rules.d/
```

Finally, launch VideoRay and other devices. See launch file for more parameters.

```
roslaunch videoray_launch videoray.launch
```

# Acknowledgments

The ROS repository for VideoRay was originally forked from [HumAnS Lab ROS Repository](https://github.com/gt-ros-pkg/humans).
