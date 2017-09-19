#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "videoray_comm/VideoRayComm.h"
#include <syllo_common/SylloNode.h>
#include <syllo_common/Filter.h>
#include <syllo_common/Orientation.h>
#include "sensor_msgs/Joy.h"
#include "videoray_control/Throttle.h"
#include "videoray_control/DesiredTrajectory.h"
#include "videoray_control/Status.h"
#include "videoray_control/UHRIComm.h"
#include "videoray_control/RTI01.h"
#include "videoray_control/RTI02.h"
#include "videoray_control/RTI03.h"
#include "videoray_control/RTI30.h"
#include "videoray_control/RTI31.h"
#include "videoray_control/RTI32.h"
#include "videoray_control/RTI33.h"
#include "videoray_control/NavData.h"

#include <opencv2/opencv.hpp>

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

ros::Subscriber throttle_sub_;
void callback_throttle( const videoray_control::Throttle& msg )
{
    vert_thrust_ = (int) msg.VertInput;
    port_thrust_ = (int) msg.PortInput;
    star_thrust_ = (int) msg.StarInput;
    ROS_INFO_STREAM("Set throttle cmd: " << msg);
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

     // TODO: TEST
     // std::cout << "axis_1 = " << axis_[1] << " , " << axis_[2] << " , " << axis_[3] << std::endl;
     if( joystick_cmd_enabled_ ) 
     {
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
     ROS_INFO_STREAM("Set desired trajecotry: " << *msg);
     desired_trajectory_ = *msg;
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
     ros::Publisher rti01_pub_ = n_.advertise<videoray_control::RTI01>("/videoray_dvl/rti01", 1);
     ros::Publisher rti02_pub_ = n_.advertise<videoray_control::RTI02>("/videoray_dvl/rti02", 1);
     ros::Publisher rti03_pub_ = n_.advertise<videoray_control::RTI03>("/videoray_dvl/rti03", 1);
     ros::Publisher rti30_pub_ = n_.advertise<videoray_control::RTI30>("/videoray_dvl/rti30", 1);
     ros::Publisher rti31_pub_ = n_.advertise<videoray_control::RTI31>("/videoray_dvl/rti31", 1);
     ros::Publisher rti32_pub_ = n_.advertise<videoray_control::RTI32>("/videoray_dvl/rti32", 1);
     ros::Publisher rti33_pub_ = n_.advertise<videoray_control::RTI33>("/videoray_dvl/rti33", 1);
     ros::Publisher rfal_dvl_vel_pub_ = n_.advertise<geometry_msgs::Vector3Stamped>("/videoray_dvl/rfal_dvl_vel", 1);
     ros::Publisher orientation_data_pub_ = n_.advertise<geometry_msgs::Vector3Stamped>("/videoray_dvl/rfal_dvl_ori", 1);
     ros::Publisher nav_data_pub_ = n_.advertise<videoray_control::NavData>("/videoray_control/nav_data", 1);

     videoray_control::RTI01 rti01_msg_;
     videoray_control::RTI02 rti02_msg_;
     videoray_control::RTI03 rti03_msg_;
     videoray_control::RTI30 rti30_msg_;
     videoray_control::RTI31 rti31_msg_;
     videoray_control::RTI32 rti32_msg_;
     videoray_control::RTI33 rti33_msg_;
     geometry_msgs::Vector3Stamped rfal_dvl_vel_msg;
     rfal_dvl_vel_msg.header.stamp = ros::Time().now();
     rfal_dvl_vel_msg.vector.x = 0.0;
     rfal_dvl_vel_msg.vector.y = 0.0;
     rfal_dvl_vel_msg.vector.z = 0.0;
     geometry_msgs::Vector3Stamped dvl_orientation_data_msg;
     geometry_msgs::PoseStamped pose_stamped_;
     geometry_msgs::TwistStamped twist_stamped_;

     videoray_control::NavData nav_data_;

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

     while (ros::ok()) 
     {
       // camera config 
       if( joystick_enabled_ ) 
       {
	 if (rising_edge(button_[8], button_prev_[8])) 
	 {
	   if( !camera_config_ ) 
	   {
	     cout << "Camera Config Entered..." << endl;
	     camera_config_ = true;
	     comm.set_cam_cmd(VideoRayComm::Enable);
	   } 
	   else 
	   {
	     cout << "Camera Config Exit..." << endl;
	     camera_config_ = false;
	   }
	 }
	    
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

	 // joystick logic for throttle commands 
	 if( rising_edge( button_[6] , button_prev_[6] ) ) 
	 {
	   joystick_cmd_enabled_ = !joystick_cmd_enabled_;
	   ROS_INFO_STREAM("Custom throttle cmd enabled: " << !joystick_cmd_enabled_);
	   
	   if( joystick_cmd_enabled_ ) 
	   {
	     std::cout << "Joystick enabled\n";
	     throttle_sub_.shutdown();
	     vert_thrust_ = 0;
	     port_thrust_ = 0;
	     star_thrust_ = 0;
	   }
	   else 
	   {
	     vert_thrust_ = 0;
	     port_thrust_ = 0;
	     star_thrust_ = 0;
	     throttle_sub_ = n_.subscribe<>("/videoray_input/throttle", 1, callback_throttle);
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

       // ramp vertical throttle commands
       if(vert_thrust_ < prev_vert_thrust)
       {
	 vert_thrust_ = prev_vert_thrust-1;
       }
       else if( vert_thrust_ > prev_vert_thrust )
       {
	 vert_thrust_ = prev_vert_thrust+1;
       }
       prev_vert_thrust = vert_thrust_;

       status = comm.set_vertical_thruster(vert_thrust_);
       status = comm.set_port_thruster(port_thrust_);
       status = comm.set_starboard_thruster(star_thrust_);
       // ROS_INFO_STREAM("vert: " << vert_thrust_ << ", port: " << port_thrust_ << ", star: " << star_thrust_);

       // publish throttle commands
       videoray_control::Throttle throttle;
       throttle.PortInput = port_thrust_;
       throttle.StarInput = star_thrust_;
       throttle.VertInput = vert_thrust_;
       throttle_pub_.publish(throttle);

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

       nav_data_.time = ros::Time::now().toSec();
       nav_data_.depth = -(double) comm.depth();
       const VideoRayComm::RTI01 &rti01 = comm.rti01();
       if (rti01.ready) {
	 nav_data_.linear_velocity.x = (double) rti01_msg_.bottom_track_velocity.x;
	 nav_data_.linear_velocity.y = (double) rti01_msg_.bottom_track_velocity.y;
	 nav_data_.linear_velocity.z = (double) rti01_msg_.bottom_track_velocity.z;
       } else {
	 nav_data_.linear_velocity.x = -99.0;
	 nav_data_.linear_velocity.y = -99.0;
	 nav_data_.linear_velocity.z = -99.0;
       }
       nav_data_.orientation.x = comm.roll();
       nav_data_.orientation.y = comm.pitch();
       nav_data_.orientation.z = comm.heading();
       nav_data_.linear_acceleration.x = comm.surge_accel();
       nav_data_.linear_acceleration.y = comm.sway_accel();
       nav_data_.linear_acceleration.z = comm.heave_accel();
       nav_data_.angular_velocity.x = comm.roll_accel();
       nav_data_.angular_velocity.y = comm.pitch_accel();
       nav_data_.angular_velocity.z = comm.yaw_accel();
       nav_data_pub_.publish(nav_data_);

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


       // dvl data processing 
       double v_body[3] = { 0.0 , 0.0 , 0.0 };

       status = comm.request_dvl_status();
       if (status != VideoRayComm::Success) {
	 cout << "DVL Transfer Error!" << endl;
       }

       if (rti01.ready) {
	 rti01_msg_.header.stamp = ros::Time().now();
	 rti01_msg_.sample_time = rti01.sample_time;
	 rti01_msg_.sample_num = rti01.sample_num;
	 rti01_msg_.temperature = rti01.temperature;
	 rti01_msg_.bottom_track_velocity.x = rti01.vector_x;
	 rti01_msg_.bottom_track_velocity.y = rti01.vector_y;
	 rti01_msg_.bottom_track_velocity.z = rti01.vector_z;
	 rti01_msg_.bottom_track_depth = rti01.bottom_track_depth;
	 rti01_msg_.water_mass_velocity.x = rti01.water_mass_x;
	 rti01_msg_.water_mass_velocity.y = rti01.water_mass_y;
	 rti01_msg_.water_mass_velocity.z = rti01.water_mass_z;
	 rti01_msg_.water_mass_depth = rti01.water_mass_depth;
	 rti01_pub_.publish(rti01_msg_);

	 v_body[0] = (double) rti01_msg_.bottom_track_velocity.x;
	 v_body[1] = (double) rti01_msg_.bottom_track_velocity.y; 
	 v_body[2] = (double) rti01_msg_.bottom_track_velocity.z;
       }

       const VideoRayComm::RTI02 &rti02 = comm.rti02();
       if (rti02.ready) {
	 rti02_msg_.header.stamp = ros::Time().now();
	 rti02_msg_.sample_time = rti02.sample_time;
	 rti02_msg_.sample_num = rti02.sample_num;
	 rti02_msg_.temperature = rti02.temperature;
	 rti02_msg_.bottom_track_velocity.x = rti02.bottom_track_x;
	 rti02_msg_.bottom_track_velocity.y = rti02.bottom_track_y;
	 rti02_msg_.bottom_track_velocity.z = rti02.bottom_track_z;
	 rti02_msg_.bottom_track_depth = rti02.bottom_track_depth;
	 rti02_msg_.water_mass_velocity.x = rti02.water_mass_x;
	 rti02_msg_.water_mass_velocity.y = rti02.water_mass_y;
	 rti02_msg_.water_mass_velocity.z = rti02.water_mass_z;
	 rti02_msg_.water_mass_depth = rti02.water_mass_depth;
	 rti02_pub_.publish(rti02_msg_);  

	 v_body[0] = (double) rti02_msg_.bottom_track_velocity.x;
	 v_body[1] = (double) rti02_msg_.bottom_track_velocity.y; 
	 v_body[2] = (double) rti02_msg_.bottom_track_velocity.z;
       }

       const VideoRayComm::RTI03 &rti03 = comm.rti03();
       if (rti03.ready) {
	 rti03_msg_.header.stamp = ros::Time().now();
	 rti03_msg_.sample_time = rti03.sample_time;
	 rti03_msg_.sample_num = rti03.sample_num;
	 rti03_msg_.temperature = rti03.temperature;
	 rti03_msg_.bottom_track_x = rti03.bottom_track_x;
	 rti03_msg_.bottom_track_y = rti03.bottom_track_y;
	 rti03_msg_.bottom_track_z = rti03.bottom_track_z;
	 rti03_msg_.bottom_track_q = rti03.bottom_track_q;
	 rti03_msg_.bottom_track_depth = rti03.bottom_track_depth;
	 rti03_msg_.water_mass_x = rti03.water_mass_x;
	 rti03_msg_.water_mass_y = rti03.water_mass_y;
	 rti03_msg_.water_mass_z = rti03.water_mass_z;
	 rti03_msg_.water_mass_q = rti03.water_mass_q;
	 rti03_msg_.water_mass_depth = rti03.water_mass_depth;
	 rti03_pub_.publish(rti03_msg_);
       }

       const VideoRayComm::RTI30 &rti30 = comm.rti30();
       if (rti30.ready) {
	 rti30_msg_.header.stamp = ros::Time().now();
	 rti30_msg_.orientation.x = rti30.heading;
	 rti30_msg_.orientation.y = rti30.pitch;
	 rti30_msg_.orientation.z = rti30.roll;
	 rti30_pub_.publish(rti30_msg_);
       }

       const VideoRayComm::RTI31 &rti31 = comm.rti31();
       if (rti31.ready) {
	 rti31_msg_.header.stamp = ros::Time().now();
	 rti31_msg_.orientation.x = rti31.heading;
	 rti31_msg_.orientation.y = rti31.pitch;
	 rti31_msg_.orientation.z = rti31.roll;
	 rti31_pub_.publish(rti31_msg_);
       }

       const VideoRayComm::RTI32 &rti32 = comm.rti32();
       if (rti32.ready) {
	 rti32_msg_.header.stamp = ros::Time().now();
	 rti32_msg_.orientation.x = rti32.vector_x;
	 rti32_msg_.orientation.y = rti32.vector_y;
	 rti32_msg_.orientation.z = rti32.vector_z;
	 rti32_msg_.pressure = rti32.pressure;
	 rti32_msg_.temperature = rti32.temperature;
	 rti32_pub_.publish(rti32_msg_);
       }

       const VideoRayComm::RTI33 &rti33 = comm.rti33();
       if (rti33.ready) {
	 rti33_msg_.header.stamp = ros::Time().now();
	 rti33_msg_.orientation.x = rti33.heading;
	 rti33_msg_.orientation.y = rti33.pitch;
	 rti33_msg_.orientation.z = rti33.roll;
	 rti33_msg_.pressure = rti33.pressure;
	 rti33_msg_.temperature = rti33.temperature;
	 rti33_pub_.publish(rti33_msg_);
       }

       // only publish dvl inertial velocity when data is valid
       if( rti01.ready || rti02.ready )
       {
	 // compute the transformation
	 double q[4] = { (double) quat.x, 
			 (double) quat.y,
			 (double) quat.z,
			 (double) quat.w }; 
	 
	 double v_inert[3] = { 0.0 , 0.0 , 0.0 };
	 rfal_quaternion_from_body_to_inertial( v_inert , v_body , q );
	 
	 // populating the inertial velocity
	 rfal_dvl_vel_msg.header.stamp = ros::Time().now();
	 rfal_dvl_vel_msg.vector.x = v_inert[0];
	 rfal_dvl_vel_msg.vector.y = v_inert[1];
	 rfal_dvl_vel_msg.vector.z = v_inert[2];
	 rfal_dvl_vel_pub_.publish( rfal_dvl_vel_msg );
       }

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
