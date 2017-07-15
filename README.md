# VideoRay ROS Packages

The ROS repository for VideoRay was originally forked from [HumAnS Lab ROS Repository](https://github.com/gt-ros-pkg/humans). Now it supports Tritech Micron sonar and USBL.

# Build

```
mkdir -p videoray/src
cd videoray/src
git clone git@bitbucket.org:stevens-rfal/videoray.git .
catkin_init_workspace
cd ..
catkin_make

source devel/setup.sh

sudo cp src/videoray_launch/scripts/99-usb-serial.rules /etc/udev/rules.d/
sudo mkdir -p /var/lib/joystick
sudo cp src/videoray_launch/scripts/joystick.state /var/lib/joystick
```

Camera

```
sudo apt-get install v4l-utils gstreamer0.10 libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
```

# Published Topics

1. `/videoray_control/pose`: `geometry_msgs/PoseStamped`
2. `/videoray_control/accelerations`: `geometry_msgs/TwistStamped`
3. `/videoray_control/throttle_cmd`: `videoray_control/Throttle`
4. `/videoray_sonar/sonarscan`: `sensor_msgs/PointCloud2`
5. `/videoray_usbl/usbl_pose`: `geometry_msgs/PoseStamped`
6. `/videoray_dvl/rti**`: RTI DVL packets

# Subscribed Topics

1. `/videoray_input/joystick`: `sensor_msgs/Joy`
2. `/videoray_input/throttle`: `videoray_control/Throttle`
3. `/videoray_input/desired_trajectory`: `videoray_control/DesiredTrajectory`
4. `/videoray_sonar/valid` `std_msgs/Bool`

# Launch

1. Change permissions `sudo rosrun videoray_launch check_dev.sh`.
2. Launch VideoRay `roslaunch videoray_launch videoray.launch`. Disable accessories as needed by `sonar:=false js:=false usbl:=false camera:=false`. If you don't disable accessories that are not connected, the program will crash because it can't find the corresponding ports.
