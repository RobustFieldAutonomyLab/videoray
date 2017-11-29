#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <videoray_msgs/Throttle.h>

videoray_msgs::Throttle cmd;
ros::Publisher cmdPub;

float maxDepth = 10.0;
float zeroThreshold = 0.1;
float inc = 0.05;
float prevVert = 0.0;

const uint8_t TRUE = 1;
const uint8_t FALSE = 0;

void ramp() {
  if (cmd.vert - prevVert > inc)
    cmd.vert = prevVert + inc;
  else if (cmd.vert - prevVert < -inc)
    cmd.vert = prevVert - inc;
  prevVert = cmd.vert;
}

/*
 *       [b6] [b7]                [b10]                  |
 *                                                       |
 *        [a1 +]                  [b8]           [a4 c+] | [b11]
 *  [a0 -]      [a0 +]        [b2]    [b9]               | [a5 u+]
 *        [a1 -]                  [b3]           [a3 c+] | [b5]
 *                                                       |
 *       [b0] [b1]                [b4]                   |
 */
void joyCallback(const sensor_msgs::JoyConstPtr &msg) {
  if (msg->axes[3] > zeroThreshold) {
    cmd.auto_depth = TRUE;
    cmd.desired_depth = maxDepth * msg->axes[3];
  } else {
    cmd.auto_depth = FALSE;
  }

  /// TODO: auto heading

  cmd.vert = msg->axes[5];
  ramp();

  cmd.port = msg->axes[1];
  cmd.star = msg->axes[1];

  float turn = msg->axes[0];
  cmd.port += turn;
  cmd.star -= turn;

  cmdPub.publish(cmd);
}

void cmdCallback(const videoray_msgs::ThrottleConstPtr &msg) {
  cmd = *msg;
  ramp();

  cmdPub.publish(cmd);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "videoray_input");
  ros::NodeHandle nh("~");

  maxDepth = nh.param("max_depth", (float) 10.0);
  cmd.auto_depth = FALSE;
  cmd.auto_heading = FALSE;
  cmd.port = 0;
  cmd.star = 0;
  cmd.vert = 0;

  std::string joyTopic = "/joy";
  std::string throttleTopic = "throttle";
  std::string cmdTopic = "command";
  ros::Subscriber joySub = nh.subscribe<>(joyTopic, 100, joyCallback);
  ros::Subscriber cmdSub = nh.subscribe<>(cmdTopic, 100, cmdCallback);

  cmdPub = nh.advertise<videoray_msgs::Throttle>(throttleTopic, 100, false);

  ros::spin();
}

