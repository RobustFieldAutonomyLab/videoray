#include "videoray_control/videoray_comm.h"

#include <ros/ros.h>
#include <videoray_msgs/NavData.h>
#include <videoray_msgs/Throttle.h>

ros::Publisher navDataPub;

const int16_t MAXRANGE = 89;
int16_t port;
int16_t star;
int16_t vert;
bool autoDepth = false;
int16_t desiredDepth;
bool autoHeading = false;
int16_t desiredHeading;

void throttleCallback(const videoray_msgs::ThrottleConstPtr &msg) {
  port = int16_t(trunc(msg->port * MAXRANGE));
  star = int16_t(trunc(msg->star * MAXRANGE));
  vert = int16_t(trunc(msg->vert * MAXRANGE));
  autoDepth = msg->auto_depth;
  if (autoDepth) {
    desiredDepth = int16_t(msg->desired_depth * 100.0 + 1000);
  }
  autoHeading = msg->auto_heading;
  if (autoHeading) {
    double a = fmod(msg->desired_heading, 360.0);
    if (a < 0)
      a += 360;

    desiredHeading = int16_t(a);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "videoray_control");
  ros::NodeHandle nh("~");


  std::string dev = nh.param("dev", std::string("/dev/videoray"));
  std::string navDataTopic = nh.param("nav_data_topic", std::string("nav_data"));

  videoray_control::VideoRayComm comm(dev);
  ros::Subscriber throttleSub = nh.subscribe<>("/videoray_input/throttle", 100, &throttleCallback);
  ros::Publisher navDataPub = nh.advertise<videoray_msgs::NavData>(navDataTopic, 100, false);

  videoray_msgs::NavData navData;
  while (ros::ok()) {
    if (autoHeading) {
      comm.set_desired_heading(desiredHeading);
    } else {
      comm.set_desired_heading(-1);
    }

    if (autoDepth) {
      comm.set_desired_depth(desiredDepth);
    } else {
      comm.set_desired_depth(-1);
    }

    comm.set_vertical_thruster(vert);
    comm.set_port_thruster(port);
    comm.set_starboard_thruster(star);

    comm.send_control_command();

    navData.stamp = ros::Time::now();
    comm.send_nav_data_command();
    comm.request_dvl_status();

    if (comm.rti01_.ready) {
      navData.linear_velocity.x = (double) comm.rti01_.bottom_track_x;
      navData.linear_velocity.y = (double) comm.rti01_.bottom_track_y;
      navData.linear_velocity.z = (double) comm.rti01_.bottom_track_z;
    } else {
      navData.linear_velocity.x = -99.0;
      navData.linear_velocity.y = -99.0;
      navData.linear_velocity.z = -99.0;
    }
    navData.depth = (float) comm.depth_;
    navData.orientation.x = comm.roll_;
    navData.orientation.y = comm.pitch_;
    navData.orientation.z = comm.heading_;
    navData.linear_acceleration.x = comm.surge_accel_;
    navData.linear_acceleration.y = comm.sway_accel_;
    navData.linear_acceleration.z = comm.heave_accel_;
    navData.angular_velocity.x = comm.roll_accel_;
    navData.angular_velocity.y = comm.pitch_accel_;
    navData.angular_velocity.z = comm.yaw_accel_;
    navDataPub.publish(navData);

    ros::spinOnce();
  }

  return 0;
}
