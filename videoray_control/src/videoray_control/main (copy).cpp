#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/Vector3.h>

#include "videoray_comm/VideoRayComm.h"
#include <syllo_common/SylloNode.h>
#include <syllo_common/Filter.h>
#include <syllo_common/Orientation.h>
#include "sensor_msgs/Joy.h"
#include "videoray_control/Throttle.h"
#include "videoray_control/DesiredTrajectory.h"
#include "videoray_control/Status.h"
#include "videoray_control/UHRIComm.h"

#include <sstream>

using std::cout;
using std::endl;

int *axis_;
int *button_;
int *button_prev_;
int num_of_buttons_ = -1;
int num_of_axes_ = -1;
bool joystick_enabled_ = false;
int prev_vert_thrust = 0;

bool joystick_cmd_enabled_ = true;
int vert_thrust_ = 0;
int port_thrust_ = 0;
int star_thrust_ = 0;

ros::Subscriber my_throttle_cmd_sub_; //= n_.subscribe<>("my_throttle_cmd", callback_my_throttle_cmd);

void callback_my_throttle_cmd(const geometry_msgs::Vector3& msg)
{
    vert_thrust_ = (int) msg.x;
    port_thrust_ = (int) msg.y;
    star_thrust_ = (int) msg.z;
}

void callback_joystick(const sensor_msgs::JoyConstPtr& msg)
{

     bool update_prev = false;

     // Check for change in number of buttons
     int num_of_buttons = msg->buttons.size();
     if (num_of_buttons != num_of_buttons_) {
          num_of_buttons_ = num_of_buttons;
          if (button_ != NULL) {
               delete[] button_;
          }
          if (button_prev_ != NULL) {
               delete[] button_prev_;
          }
          button_ = new int[num_of_buttons_];
          button_prev_ = new int[num_of_buttons_];
          update_prev = true;
     }

     // Check for change in number of axes
     int num_of_axes = msg->axes.size();
     if (num_of_axes != num_of_axes_) {
          num_of_axes_ = num_of_axes;
          if (axis_ != NULL) {
               delete[] axis_;
          }
          axis_ = new int[num_of_axes_];
     }

     // Update button array
     for (int i = 0 ; i < num_of_buttons; i++) {
          button_[i] = msg->buttons[i];
     }

     if (update_prev) {
          memcpy(button_prev_,button_,sizeof(int)*num_of_buttons_);
     }

     // Update axis array
     for (int i = 0 ; i < num_of_axes; i++) {
          axis_[i] = msg->axes[i];
     }

     if (joystick_cmd_enabled_) {
         vert_thrust_ = invert_sign(normalize(axis_[1], -32767, 32767, -99, 99));
         port_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));
         star_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));

         int turn_ = invert_sign(normalize(axis_[2], -32767, 32767, -89, 89));
         port_thrust_ -= turn_;
         star_thrust_ += turn_;
     }

     joystick_enabled_ = true;
}

videoray_control::DesiredTrajectory desired_trajectory_;

void callback_desired_trajectory(const videoray_control::DesiredTrajectoryConstPtr& msg)
{
     desired_trajectory_ = *msg;
}

geometry_msgs::Vector3 hold_cmd;

void callback_hold_cmd(const geometry_msgs::Vector3 &msg) {
  hold_cmd = msg;
}

videoray_control::UHRIComm uhri_comm_;
void callback_uhri_comm(const videoray_control::UHRICommConstPtr& msg)
{
     uhri_comm_ = *msg;
}

int rising_edge(int curr, int prev)
{
     if (curr && (curr != prev)) {
          return 1;
     } else {
          return 0;
     }
}

typedef enum TaskComplete {
     tk_idle,
     tk_inc,
     tk_dec
}TaskComplete_t;

typedef enum NoAck {
     na_idle,
     na_on_0,
     na_off_0,
     na_on_1
}NoAck_t;

typedef enum Ack {
     a_idle,
     a_on_0
}Ack_t;

typedef enum Attention {
     att_idle,
     att_on,
     att_off
}Attention_t;
#define ATT_FLASH_COUNT 10

#define LIGHTS_ON  63
#define LIGHTS_OFF 0

#define BLINK_ON_DUR (2.0/10.0*1e9)
#define BLINK_OFF_DUR (2.0/10.0*1e9)

int main(int argc, char **argv)
{

     ros::init(argc, argv, "videoray_control");
     ros::NodeHandle n_("~");

     // Handle generic parameters for a SylloNode
     SylloNode syllo_node_;
     syllo_node_.init();

     ros::Subscriber joystick_sub_ = n_.subscribe<>("/videoray_input/joystick", 1, callback_joystick);
     ros::Subscriber autopilot_sub_ = n_.subscribe<>("/videoray_input/desired_trajectory", 1, callback_desired_trajectory);
     ros::Subscriber uhri_comm_sub_ = n_.subscribe<>("uhri_comm", 1, callback_uhri_comm);
     ros::Publisher pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("pose", 1);
     ros::Publisher throttle_pub_ = n_.advertise<videoray_control::Throttle>("throttle_cmd", 1);
     ros::Publisher twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("accelerations",1);
     ros::Publisher videoray_status_pub_ = n_.advertise<videoray_control::Status>("videoray_status",1);

     ros::Subscriber hold_cmd_sub = n_.subscribe<>("/videoray_input/hold_cmd", 1, callback_hold_cmd);
     hold_cmd.x = 0;
     hold_cmd.y = 0;
     hold_cmd.z = 0;

     geometry_msgs::PoseStamped pose_stamped_;
     geometry_msgs::TwistStamped twist_stamped_;

     std::string videoray_port;
     n_.param<std::string>("port", videoray_port, "/dev/videoray");
     std::cout << "VideoRay USB Port: " << videoray_port << std::endl;
     VideoRayComm::Status_t status;
     VideoRayComm comm(videoray_port);

     videoray_control::Status videoray_status_;
     videoray_status_.water_temp = 0;
     videoray_status_.tether_voltage = 0;
     videoray_status_.voltage_12V = 0;
     videoray_status_.current_12V = 0;
     videoray_status_.internal_temp = 0;
     videoray_status_.internal_relative_humidity = 0;
     videoray_status_.comm_err_count = 0;
     videoray_status_.firmware_version = 0;

     int lights_ = LIGHTS_OFF;
     int tilt_ = 0;
     int focus_ = 0;
     bool camera_config_ = false;
     bool enable_log_ = false;
     VideoRayComm::ManipState_t manip_state_ = VideoRayComm::Idle;

     bool task_complete_in_prog_ = false;
     TaskComplete_t tk_sm_ = tk_idle;
     NoAck_t na_sm_ = na_idle;
     Ack_t a_sm_ = a_idle;
     Attention_t att_sm_ = att_idle;
     int att_count = 0;

     uhri_comm_.Msg2Diver = videoray_control::UHRIComm::NONE;


     ros::Time timer_;

     while (ros::ok()) {

          if (joystick_enabled_) {
               if (rising_edge(button_[8], button_prev_[8])) {
                    if (!camera_config_) {
                         cout << "Camera Config Entered..." << endl;
                         camera_config_ = true;
                         comm.set_cam_cmd(VideoRayComm::Enable);
                    } else {
                         cout << "Camera Config Exit..." << endl;
                         camera_config_ = false;
                    }
               }

            //    vert_thrust_ = invert_sign(normalize(axis_[1], -32767, 32767, -99, 99));
            //    port_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));
            //    star_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));

            //    turn_ = invert_sign(normalize(axis_[2], -32767, 32767, -89, 89));
            //    port_thrust_ -= turn_;
            //    star_thrust_ += turn_;

               tilt_ += invert_sign(normalize(axis_[5], -32767, 32767, -1, 1));
               tilt_ = saturate(tilt_, -80, 80);

               focus_ += invert_sign(normalize(axis_[4], -32767, 32767, -1, 1));
               focus_ = saturate(tilt_, -100, 100);

               if (!camera_config_) {
                    if (button_[3]) {
                         lights_ += 1;
                    } else if (button_[1]) {
                         lights_ -= 1;
                    }
                    lights_ = saturate(lights_, LIGHTS_OFF, LIGHTS_ON);
               } else {
                    if (rising_edge(button_[3], button_prev_[3])) {
                         comm.set_cam_cmd(VideoRayComm::Arrow_Up);
                    } else if (rising_edge(button_[1], button_prev_[1])) {
                         comm.set_cam_cmd(VideoRayComm::Arrow_Down);
                    } else if (rising_edge(button_[0], button_prev_[0])) {
                         comm.set_cam_cmd(VideoRayComm::Arrow_Left);
                    } else if (rising_edge(button_[2], button_prev_[2])) {
                         comm.set_cam_cmd(VideoRayComm::Arrow_Right);
                    }
               }

               // Handle joystick buttons for opening / closing manipulator
              //  if (button_[5]) { // Open manipulator
              //       manip_state_ = VideoRayComm::Opening;
              //  } else if (button_[7]) { // Close manipulator
              //       manip_state_ = VideoRayComm::Closing;
              //  } else { // Idle manipulator
              //       manip_state_ = VideoRayComm::Idle;
              //  }

                if (rising_edge(button_[6], button_prev_[6])) {
                    joystick_cmd_enabled_ = !joystick_cmd_enabled_;
                    ROS_INFO_STREAM("Custom throttle cmd enabled: " << !joystick_cmd_enabled_);
                    if (joystick_cmd_enabled_) {
                        my_throttle_cmd_sub_.shutdown();
                        vert_thrust_ = 0;
                        port_thrust_ = 0;
                        star_thrust_ = 0;
                    }
                    else {
                        vert_thrust_ = 0;
                        port_thrust_ = 0;
                        star_thrust_ = 0;
                        my_throttle_cmd_sub_ = n_.subscribe<>("/videoray_input/throttle_cmd", 100, callback_my_throttle_cmd);
                   }
                }

               // Reset button
               if (button_[9]) {
                    tilt_ = 0;
                    focus_ = 0;
                    lights_ = LIGHTS_OFF;
                    port_thrust_ = 0;
                    star_thrust_ = 0;
                    vert_thrust_ = 0;
                    prev_vert_thrust =0;
               }

               memcpy(button_prev_,button_,sizeof(int)*num_of_buttons_);
          }

          // Handle UHRI Comms to control lights
          if (uhri_comm_.Msg2Diver != videoray_control::UHRIComm::NONE) {
               if (uhri_comm_.Msg2Diver == videoray_control::UHRIComm::ACK) {
                    a_sm_ = a_on_0;
               } else if (uhri_comm_.Msg2Diver == videoray_control::UHRIComm::NACK) {
                    na_sm_ = na_on_0;
               } else if (uhri_comm_.Msg2Diver == videoray_control::UHRIComm::ATTENTION) {
                    att_sm_ = att_on;
                    att_count = 0;
               } else if (uhri_comm_.Msg2Diver == videoray_control::UHRIComm::TASK_COMPLETE) {
                    tk_sm_ = tk_inc;
               }

               // Reset Msg2Diver to NONE, so it doesn't trigger next iteration
               uhri_comm_.Msg2Diver = videoray_control::UHRIComm::NONE;

               lights_ = LIGHTS_OFF;
               timer_ = ros::Time::now();
          }

          // Attention State Machine
          switch(att_sm_) {
          case att_on:
               lights_ = LIGHTS_ON;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_ON_DUR)) {
                    att_sm_ = att_off;
                    timer_ = ros::Time::now();
               }
               break;
          case att_off:
               lights_ = LIGHTS_OFF;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_OFF_DUR)) {
                    att_sm_ = att_on;
                    timer_ = ros::Time::now();
                    att_count++;

                    if (att_count >= ATT_FLASH_COUNT) {
                         att_sm_ = att_idle;
                         att_count = 0;
                    }
               }
               break;
          default:
               break;
          }

          // No Ack State Machine
          switch(na_sm_) {
          case na_on_0:
               lights_ = LIGHTS_ON;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_ON_DUR)) {
                    na_sm_ = na_off_0;
                    timer_ = ros::Time::now();
               }
               break;
          case na_off_0:
               lights_ = LIGHTS_OFF;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_OFF_DUR)) {
                    na_sm_ = na_on_1;
                    timer_ = ros::Time::now();
               }
               break;
          case na_on_1:
               lights_ = LIGHTS_ON;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_ON_DUR)) {
                    na_sm_ = na_idle;
                    timer_ = ros::Time::now();
                    lights_ = LIGHTS_OFF;
               }
               break;
          default:
               break;
          }

          // Ack State Machine
          switch(a_sm_) {
          case a_on_0:
               lights_ = LIGHTS_ON;
               if (( ros::Time::now() - timer_) > ros::Duration(0,BLINK_ON_DUR)) {
                    a_sm_ = a_idle;
                    timer_ = ros::Time::now();
                    lights_ = LIGHTS_OFF;
               }
               break;
          default:
               break;
          }

          // Task Complete State Machine
          switch(tk_sm_) {
          case tk_inc:
               lights_ += 1;
               lights_ = saturate(lights_, LIGHTS_OFF, LIGHTS_ON);
               if (lights_ >= LIGHTS_ON) {
                    tk_sm_ = tk_dec;
               }
               break;
          case tk_dec:
               lights_ -= 1;
               lights_ = saturate(lights_, LIGHTS_OFF, LIGHTS_ON);
               if (lights_ <= 0) {
                    tk_sm_ = tk_idle;
               }
               break;
          default:
               break;
          }

          if ((int)round(hold_cmd.z) == 1) {
               comm.set_desired_heading(hold_cmd.x);
               comm.set_desired_depth(-1);
          } else if ((int)round(hold_cmd.z) == 2) {
               comm.set_desired_heading(-1);
               comm.set_desired_depth(hold_cmd.y * 100 + 1000);
          } else if ((int)round(hold_cmd.z) == 3) {
               comm.set_desired_heading(hold_cmd.x);
               comm.set_desired_depth(hold_cmd.y * 100 + 1000);
          } else if ((int)round(hold_cmd.z) == 0) {
               comm.set_desired_depth(-1);
               comm.set_desired_heading(-1);
               
            // Handle Auto heading command
            if (desired_trajectory_.heading_enabled) {
                 comm.set_desired_heading(desired_trajectory_.heading);
            } else {
                 // Disable auto heading
                 comm.set_desired_heading(-1);
            }

            // Handle Auto Depth Command
            if (desired_trajectory_.depth_enabled) {
                 comm.set_desired_depth(desired_trajectory_.depth*100+1000);
            } else {
                 // Disable auto depth
                 comm.set_desired_depth(-1);
            }

          }

          if( vert_thrust_ == 0)
               vert_thrust_ = 0;
          else if(vert_thrust_ < prev_vert_thrust)
          {
               vert_thrust_ = prev_vert_thrust-1;
          }
          else if (vert_thrust_>prev_vert_thrust)
          {
               vert_thrust_ = prev_vert_thrust+1;
          }
          prev_vert_thrust = vert_thrust_;

          status = comm.set_vertical_thruster(vert_thrust_);
          status = comm.set_port_thruster(port_thrust_);
          status = comm.set_starboard_thruster(star_thrust_);
          status = comm.set_lights(lights_);
          status = comm.set_camera_tilt(tilt_);

          status = comm.send_control_command();
          if (status != VideoRayComm::Success) {
               cout << "Exec Transfer Error!" << endl;
          }

          status = comm.send_nav_data_command();
          if (status != VideoRayComm::Success) {
              cout << "Exec Transfer Error!" << endl;
          }

          status = comm.set_manipulator_state(manip_state_);
          if (status != VideoRayComm::Success) {
              cout << "Error: Set Manipulator State" << endl;
          }

          // Time stamp the message
          pose_stamped_.header.stamp = ros::Time().now();
          pose_stamped_.header.frame_id = "videoray";
          twist_stamped_.header.stamp = ros::Time().now();
          twist_stamped_.header.frame_id = "videoray";

          // Populate x,y,z positions
          pose_stamped_.pose.position.x = 0;
          pose_stamped_.pose.position.y = 0;
          pose_stamped_.pose.position.z = -comm.depth();

          // Populate the orientation
          geometry_msgs::Quaternion quat;
          eulerToQuaternion_xyzw_deg(comm.roll(), comm.pitch(), comm.heading(),
                                    quat.x, quat.y, quat.z, quat.w);

          pose_stamped_.pose.orientation = quat;

          // Publish pose stamped and regular pose for rqt_pose_view
          pose_pub_.publish(pose_stamped_);

          videoray_control::Throttle throttle;
          throttle.PortInput = port_thrust_;
          throttle.StarInput = star_thrust_;
          throttle.VertInput = vert_thrust_;
          throttle_pub_.publish(throttle);

          // Linear accelerations
          twist_stamped_.twist.linear.x = comm.surge_accel();
          twist_stamped_.twist.linear.y = comm.sway_accel();
          twist_stamped_.twist.linear.z = comm.heave_accel();

          // Angular accelerations
          twist_stamped_.twist.angular.x = comm.roll_accel();
          twist_stamped_.twist.angular.y = comm.pitch_accel();
          twist_stamped_.twist.angular.z = comm.yaw_accel();

          twist_pub_.publish(twist_stamped_);

          status = comm.request_status();
          if (status != VideoRayComm::Success) {
              cout << "Exec Transfer Error!" << endl;
          }

          videoray_status_.internal_relative_humidity = comm.humidity();
          videoray_status_.water_temp = comm.water_temperature();
          videoray_status_.voltage_12V = comm.rov_voltage();

          videoray_status_pub_.publish(videoray_status_);

          //cout << "------------------------------------" << endl;
          //cout << "Heading: " << comm.heading() << endl;
          //cout << "Pitch: " << comm.pitch() << endl;
          //cout << "Roll: " << comm.roll() << endl;
          //cout << "Depth: " << comm.depth() << endl;
          //cout << "Yaw Accel: " << comm.yaw_accel() << endl;
          //cout << "Pitch Accel: " << comm.pitch_accel() << endl;
          //cout << "Roll Accel: " << comm.roll_accel() << endl;
          //cout << "Surge Accel: " << comm.surge_accel() << endl;
          //cout << "Sway Accel: " << comm.sway_accel() << endl;
          //cout << "Heave Accel: " << comm.heave_accel() << endl;
          //}

          syllo_node_.spin();
     }

     // Clean up syllo node
     syllo_node_.cleanup();

     return 0;
}
