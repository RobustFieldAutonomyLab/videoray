# VideoRay ROS Packages

The ROS repository for VideoRay was originally forked from [HumAnS Lab ROS Repository](https://github.com/gt-ros-pkg/humans). Now it supports Tritech Micron sonar and USBL.

# Build

```
mkdir -p videoray/src
cd videoray/src
catkin_init_workspace
git clone git@bitbucket.org:stevens-rfal/videoray.git
cd ..
catkin_make

source devel/setup.sh
sudo cp src/videoray_launch/scripts/99-usb-serial.rules /etc/udev/rules.d/
sudo cp src/videoray_launch/scripts/joystick.state /var/lib/joystick
```

# Published Topics

1. `/videoray_control/pose`: `geometry_msgs/PoseStamped`
2. `/videoray_control/accelerations`: `geometry_msgs/TwistStamped`
3. `/videoray_control/throttle_cmd`: `videoray_control/Throttle`
4. `/videoray_sonar/sonarscan`: `sensor_msgs/PointCloud2`
5. `/videoray_usbl/usbl_pose`: `geometry_msgs/PoseStamped`

# Subscribed Topics

1. `/videoray_input/joystick`: `sensor_msgs/Joy`
2. `/videoray_input/throttle`: `videoray_control/Throttle`
3. `/videoray_input/desired_trajectory`: `videoray_control/DesiredTrajectory`
4. `/videoray_sonar/valid` `std_msgs/Bool`
