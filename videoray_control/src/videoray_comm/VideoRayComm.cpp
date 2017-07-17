#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <sstream>
#include <math.h>
#include <opencv2/opencv.hpp>


#include "videoray_comm/VideoRayComm.h"

//////////////////////////////
// TX Control Packet Defines
//////////////////////////////
#define PORT_THRUST_LSB  0
#define PORT_THRUST_MSB  1
#define STAR_THRUST_LSB  2
#define STAR_THRUST_MSB  3
#define VERT_THRUST_LSB  4
#define VERT_THRUST_MSB  5
#define LIGHTS_LSB       6
#define CAM_TILT         7
#define CAM_FOCUS        8
#define UNKNOWN_0        9  // some sort of camera control and camera menu 
#define UNKNOWN_1        10 // selection
#define AUTO_DEPTH_LSB   11
#define AUTO_DEPTH_MSB   12
#define AUTO_HEADING_LSB 13
#define AUTO_HEADING_MSB 14

#define TX_CTRL_SIZE     15

#define AUTO_DEPTH_P_LSB 38
#define AUTO_DEPTH_P_MSB 39
#define AUTO_DEPTH_I_LSB 40
#define AUTO_DEPTH_I_MSB 41
#define AUTO_DEPTH_D_LSB 42
#define AUTO_DEPTH_D_MSB 43
#define AUTO_DEPTH_K_LSB 44
#define AUTO_DEPTH_K_MSB 45

// removed defines taht weren't used AUTO_HEADING_P/I/D/K_LSB


////////////////////////////////
//// RX Control Packet Defines
////////////////////////////////
//#define UNKNOWN_2   0
//#define HEADING_LSB 1
//#define HEADING_MSB 2
//#define PITCH_LSB   3
//#define PITCH_MSB   4
//#define ROLL_LSB    5
//#define ROLL_MSB    6

//////////////////////////////
// RX Control Packet Defines
// CSR Map Defines
////////////////////////////
//#define DEVICE_ID      0
//#define DEPTH_LSB      1
//#define DEPTH_MSB      2
//#define HEADING_LSB    3
//#define HEADING_MSB    4
//#define PITCH_LSB      5
//#define PITCH_MSB      6
//#define ROLL_LSB       7
//#define ROLL_MSB       8
//#define YAW_ACC_LSB    9
//#define YAW_ACC_MSB    10
//#define PITCH_ACC_LSB  11
//#define PITCH_ACC_MSB  12
//#define ROLL_ACC_LSB   13
//#define ROLL_ACC_MSB   14
//#define SURGE_ACC_LSB  15
//#define SURGE_ACC_MSB  16
//#define SWAY_ACC_LSB   17
//#define SWAY_ACC_MSB   18
//#define HEAVE_ACC_LSB  19
//#define HEAVE_ACC_MSB  20

#define DEVICE_ID      0
#define HEADING_LSB    1
#define HEADING_MSB    2
#define PITCH_LSB      3
#define PITCH_MSB      4
#define ROLL_LSB       5
#define ROLL_MSB       6
#define DEPTH_LSB      7
#define DEPTH_MSB      8
#define YAW_ACC_LSB    9
#define YAW_ACC_MSB    10
#define PITCH_ACC_LSB  11
#define PITCH_ACC_MSB  12
#define ROLL_ACC_LSB   13
#define ROLL_ACC_MSB   14
#define SURGE_ACC_LSB  15
#define SURGE_ACC_MSB  16
#define SWAY_ACC_LSB   17
#define SWAY_ACC_MSB   18
#define HEAVE_ACC_LSB  19
#define HEAVE_ACC_MSB  20
#define RAW_MAG_X_LSB  21
#define RAW_MAG_X_MSB  22
#define RAW_MAG_Y_LSB  23
#define RAW_MAG_Y_MSB  24
#define RAW_MAG_Z_LSB  25
#define RAW_MAG_Z_MSB  26
#define ATTITUDE_LSB   27
#define ATTITUDE_MSB   28
//#define HUMIDITY_LSB   29
//#define HUMIDITY_MSB   30
//#define WATER_TEMP_LSB 31
//#define WATER_TEMP_MSB 32
//#define ROV_PWR_LSB    33
//#define ROV_PWR_MSB    34

#define WATER_TEMP_LSB      00
#define WATER_TEMP_MSB      01
#define TETHER_VOLT_LSB     02
#define TETHER_VOLT_MSB     03
#define VOLTAGE_12V_LSB     04
#define VOLTAGE_12V_MSB     05
#define CURRENT_12V_LSB     06
#define CURRENT_12V_MSB     07
#define INTERNAL_TEMP_LSB   08
#define INTERNAL_TEMP_MSB   09
#define HUMIDITY_LSB        10 
#define HUMIDITY_MSB        11
#define COMM_ERR_COUNT_LSB  12
#define COMM_ERR_COUNT_MSB  13


#define ROV_PWR_LSB    33
#define ROV_PWR_MSB    34


using std::cout;
using std::endl;

VideoRayComm::VideoRayComm(std::string port)
{
     packetizer_.set_network_id(0x01);
     
     int status;
     status = serial_.Open(port.c_str(), 115200);
     if (status != 1) {
     	  cout << "Error while opening port. Permission problem ?" << endl;
     	  exit(-1);
     }
     serial_.FlushReceiver();

     tx_ctrl_data[PORT_THRUST_LSB] = 0;
     tx_ctrl_data[PORT_THRUST_MSB] = 0;
     tx_ctrl_data[STAR_THRUST_LSB] = 0;
     tx_ctrl_data[STAR_THRUST_MSB] = 0;
     tx_ctrl_data[VERT_THRUST_LSB] = 0;
     tx_ctrl_data[VERT_THRUST_MSB] = 0;
     tx_ctrl_data[LIGHTS_LSB] = 0;
     tx_ctrl_data[CAM_TILT] = 0;
     tx_ctrl_data[CAM_FOCUS] = 0;
     tx_ctrl_data[UNKNOWN_0] = 0x0;
     tx_ctrl_data[UNKNOWN_1] = 0x0;
     tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
     tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     
     heading_ = 0;
     pitch_ = 0;
     roll_ = 0;

     // set_depth_pid_parameters();

     manip_state_ = VideoRayComm::Idle;
}

VideoRayComm::~VideoRayComm()
{
     serial_.Close();
}

// Hard coded definitions for manipulator command
char open_manip_data[] = {0x35,0x49,0x0,0x0,0x0,0x0,0x3,0x0};
char close_manip_data[] = {0x35,0x49,0x0,0x0,0x0,0x0,0x2,0x0};
char idle_manip_data[] = {0x35,0x49,0x0,0x0,0x0,0x0,0x0,0x0};
VideoRayComm::Status_t VideoRayComm::set_manipulator_state(VideoRayComm::ManipState_t state)
{
     // If the manipulator is already set to the correct state,
     // return successfully.
     if (manip_state_ == state ) {
          return VideoRayComm::Success;
     }
     
     manip_state_ = state;
     
     char * packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_network_id(0x42);
     packetizer_.set_flags(0x00);
     packetizer_.set_csr_addr(0xF0);          

     if (manip_state_ == VideoRayComm::Opening) {
          packetizer_.set_data(open_manip_data, MANIP_CTRL_SIZE);
     } else if (manip_state_ == VideoRayComm::Closing) {
          packetizer_.set_data(close_manip_data, MANIP_CTRL_SIZE);
     } else {
          packetizer_.set_data(idle_manip_data, MANIP_CTRL_SIZE);
     }

     bytes = packetizer_.generate_packet(&packet);    

     //for(int i = 0; i < bytes; i++) {
     //     printf("%02X ",(unsigned char)packet[i]);
     //}
     //printf("\n");
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);     

     return VideoRayComm::Success;
}

#define CAM_CTRL_SIZE 2
char enable_cam_menu[] =  {0xCA, 0x01};
char arrow_right_data[] = {0xCA, 0x10};
char arrow_left_data[] = {0xCA, 0x08};
char arrow_down_data[] = {0xCA, 0x04};
char arrow_up_data[] = {0xCA, 0x02};

VideoRayComm::Status_t VideoRayComm::set_cam_cmd(VideoRayComm::CamCtrl_t cam_ctrl)
{
     char * packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_network_id(0x01);
     packetizer_.set_flags(0x01);
     packetizer_.set_csr_addr(0xF0);          

     if (cam_ctrl == VideoRayComm::Arrow_Up) {
          packetizer_.set_data(arrow_up_data, CAM_CTRL_SIZE);
     } else if (cam_ctrl == VideoRayComm::Arrow_Right) {
          packetizer_.set_data(arrow_right_data, CAM_CTRL_SIZE);
     } else if (cam_ctrl == VideoRayComm::Arrow_Down) {
          packetizer_.set_data(arrow_down_data, CAM_CTRL_SIZE);
     } else if (cam_ctrl == VideoRayComm::Arrow_Left) {
          packetizer_.set_data(arrow_left_data, CAM_CTRL_SIZE);
     } else if (cam_ctrl == VideoRayComm::Enable) {
          packetizer_.set_data(enable_cam_menu, CAM_CTRL_SIZE);
     } 

     bytes = packetizer_.generate_packet(&packet);    

     //for(int i = 0; i < bytes; i++) {
     //     printf("%02X ",(unsigned char)packet[i]);
     //}
     //printf("\n");

     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);     

     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");                              
     } else {
          printf("Cam Control - Decode Error.\n");
          printf("Status: %d\n", status);
          printf("Flushing receiver.\n");
          serial_.FlushReceiver();
     }

     return VideoRayComm::Success;}

VideoRayComm::Status_t VideoRayComm::set_desired_heading(int heading)
{
     if (heading > 360 || heading < 0) {
          tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
          tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_HEADING_LSB] = heading & 0x00FF;
          tx_ctrl_data[AUTO_HEADING_MSB] = (heading & 0xFF00) >> 8;
     }

     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_desired_depth(int depth)
{
     if (depth < 0) {
          tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_DEPTH_LSB] = depth & 0x00FF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = (depth & 0xFF00) >> 8;
     }
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_focus(int focus)
{
     tx_ctrl_data[CAM_FOCUS] = focus;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_camera_tilt(int tilt)
{
     tx_ctrl_data[CAM_TILT] = tilt;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_lights(int lights)
{
     tx_ctrl_data[LIGHTS_LSB] = lights;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_vertical_thruster(int thrust)
{
     tx_ctrl_data[VERT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[VERT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_port_thruster(int thrust)
{
     tx_ctrl_data[PORT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[PORT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_starboard_thruster(int thrust)
{
     tx_ctrl_data[STAR_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[STAR_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::send_control_command()
{
     char * packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_network_id(0x01);
     packetizer_.set_flags(0x03);
     packetizer_.set_csr_addr(0x00);          
     packetizer_.set_data(tx_ctrl_data, TX_CTRL_SIZE);
     bytes = packetizer_.generate_packet(&packet);    
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     int test = 0;
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }       
          test++;
          if (test > 100) break;
     } while(status == Packetizer::In_Progress);
     
     //if (status == Packetizer::Success) {
     //     bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
          //short temp = 0;
          //temp = ((short)(packet[HEADING_MSB]) << 8) | packet[HEADING_LSB]; 
          //heading_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[PITCH_MSB]) << 8) | packet[PITCH_LSB]; 
          //pitch_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[ROLL_MSB]) << 8) | packet[ROLL_LSB]; 
          //roll_ = temp;
                    
     //} else {
     //     printf("Control Command - Decode Error.\n");
     //     printf("Status: %d\n", status);
     //     printf("Flushing receiver.\n");
     //     serial_.FlushReceiver();
     //}
     serial_.FlushReceiver();
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_depth_pid_parameters()
{
     char * packet;
     int bytes;
     
     // Holds data for addresses 0x26 through 0x2D
     // You should pass the values for these PID variables as parameters
     // into this function.
     char data[8];
     data[AUTO_DEPTH_P_LSB] = 1; // arbitrary data.
     data[AUTO_DEPTH_P_MSB] = 0; // arbitrary data.
     data[AUTO_DEPTH_I_LSB] = 0; // arbitrary data.
     data[AUTO_DEPTH_I_MSB] = 0; // arbitrary data.
     data[AUTO_DEPTH_D_LSB] = 0; // arbitrary data.
     data[AUTO_DEPTH_D_MSB] = 0; // arbitrary data.
     data[AUTO_DEPTH_K_LSB] = 1; // arbitrary data.
     data[AUTO_DEPTH_K_MSB] = 0; // arbitrary data.
     
     // Generate Packet and grab reference to it
     packetizer_.set_network_id(0x01); // Network ID for videoray
     packetizer_.set_flags(0x00); // No response expected
     packetizer_.set_csr_addr(0x26);          
     
     packetizer_.set_data(data, 8);
     bytes = packetizer_.generate_packet(&packet);    

     // You might want to print out the contents of packet at this point and
     // make sure that all of the bytes look correct as far as the
     // communication protocol is concerned.
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     int test = 0;
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }       
          test++;
          if (test > 100) break;
     } while(status == Packetizer::In_Progress);
     
     //if (status == Packetizer::Success) {
     //     bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
          //short temp = 0;
          //temp = ((short)(packet[HEADING_MSB]) << 8) | packet[HEADING_LSB]; 
          //heading_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[PITCH_MSB]) << 8) | packet[PITCH_LSB]; 
          //pitch_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[ROLL_MSB]) << 8) | packet[ROLL_LSB]; 
          //roll_ = temp;
                    
     //} else {
     //     printf("Control Command - Decode Error.\n");
     //     printf("Status: %d\n", status);
     //     printf("Flushing receiver.\n");
     //     serial_.FlushReceiver();
     //}
     serial_.FlushReceiver();
     return VideoRayComm::Success;
}


short VideoRayComm::swap_bytes(char *array, int msb, int lsb)
{
     short temp = 0;
     temp = ((unsigned short)(array[msb] & 0xFF) << 8) | (array[lsb] & 0xFF);
     return temp;
}

VideoRayComm::Status_t VideoRayComm::send_nav_data_command()
{
     char * packet;
     int bytes;

     //////////////////////
     // Tx Sensor Message
     //////////////////////
     // Generate Packet and grab reference to it
     //// Packet for CSR map read...
     //packetizer_.set_flags(0x94);
     //packetizer_.set_csr_addr(0x66);          
     //packetizer_.set_data(tx_ctrl_data, 0);
     
     // Navigation data vendor specific message...
     packetizer_.set_network_id(0x01);
     packetizer_.set_flags(0x5);
     packetizer_.set_csr_addr(0x0);          
     packetizer_.set_data(tx_ctrl_data, 0);

     bytes = packetizer_.generate_packet(&packet);
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");

          heading_ = swap_bytes(packet, HEADING_MSB, HEADING_LSB) / 10.0;
          pitch_ = swap_bytes(packet, PITCH_MSB, PITCH_LSB) / 10.0;
          roll_ = swap_bytes(packet, ROLL_MSB, ROLL_LSB) / 10.0;

          depth_ = (swap_bytes(packet, DEPTH_MSB, DEPTH_LSB) - 1000.0) / 100.0;

          yaw_accel_ = swap_bytes(packet, YAW_ACC_MSB, YAW_ACC_LSB) / 1000.0;
          pitch_accel_ = swap_bytes(packet, PITCH_ACC_MSB, PITCH_ACC_LSB) / 1000.0;
          roll_accel_ = swap_bytes(packet, ROLL_ACC_MSB, ROLL_ACC_LSB) / 1000.0;

          surge_accel_ = swap_bytes(packet, SURGE_ACC_MSB, SURGE_ACC_LSB) / 1000.0;
          sway_accel_ = swap_bytes(packet, SWAY_ACC_MSB, SWAY_ACC_LSB) / 1000.0;          
          heave_accel_ = swap_bytes(packet, HEAVE_ACC_MSB, HEAVE_ACC_LSB) / 1000.0;          
          
     } else {
          printf("Nav Data - Decode Error.\n");
          printf("Status: %d\n", status);
          printf("Flushing receiver.\n");
          serial_.FlushReceiver();
     }
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::request_status()
{
     char * packet;
     int bytes;

     packetizer_.set_network_id(0x01);
     packetizer_.set_flags(0x8E);
     packetizer_.set_csr_addr(0x7A);
     packetizer_.set_data(tx_ctrl_data, 0);

     bytes = packetizer_.generate_packet(&packet);
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
                   
          rov_voltage_ = swap_bytes(packet,  VOLTAGE_12V_MSB, VOLTAGE_12V_LSB);
          water_temperature_ = swap_bytes(packet, WATER_TEMP_MSB, WATER_TEMP_LSB);
          humidity_ = swap_bytes(packet, HUMIDITY_MSB, HUMIDITY_LSB);
          
     } else {
          printf("ROV Status - Decode Error.\n");
          printf("Status: %d\n", status);
          printf("Flushing receiver.\n");
          serial_.FlushReceiver();
     }
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::request_dvl_status()
{
     rti01_.ready = false;
     rti02_.ready = false;
     rti03_.ready = false;
     rti30_.ready = false;
     rti31_.ready = false;
     rti32_.ready = false;
     rti33_.ready = false;
     pose_data.ready = false;
     orientation_data.ready = false;

     char * packet;
     int bytes;

     packetizer_.set_network_id(0x0A);
     packetizer_.set_flags(0x00); // Ignored
     packetizer_.set_csr_addr(0xF0);
     packetizer_.set_data(tx_ctrl_data, 0);

     bytes = packetizer_.generate_packet(&packet);
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          for (int x = 0 ; x < bytes ; x++) {
               if (packet[x] == 16 || packet[x] == '\r' || packet[x] == 0)
                    continue;
               dvl_buff_.push_back(packet[x]);
          }

          while (true) {
               std::vector<char>::iterator end = std::find(dvl_buff_.begin(), dvl_buff_.end(), '\n');
               if (end == dvl_buff_.end())
                    break;

               std::string str(dvl_buff_.begin(), end);
               parse_rti_dvl_data(str);
               dvl_buff_.erase(dvl_buff_.begin(), end + 1);
          }
      } else {
          printf("DVL Status - Decode Error.\n");
          printf("Status: %d\n", status);
          printf("Flushing receiver.\n");
          serial_.FlushReceiver();
     }
     return VideoRayComm::Success;
}

void VideoRayComm::parse_rti_dvl_data(const std::string &str) {
     if (str[0] != '$') {
          std::cout << "Packet doesn't start with $: " << int(str[0]) << ", " << str << std::endl;
          return;
     }

    std::stringstream ss;
    ss.str(str);
    std::string field;
    std::vector<std::string> data_fields;
    while (std::getline(ss, field, ',')) {
         data_fields.push_back(field);
    }

    int rti = std::atoi(str.c_str() + 5);
    switch (rti) {
     case 01:
          if (data_fields.size() != 15) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti01_.sample_time = std::atoi(data_fields[1].c_str()) / 100.0;
          rti01_.sample_num = std::atoi(data_fields[2].c_str());
          rti01_.temperature = std::atoi(data_fields[3].c_str()) / 100.0;
          rti01_.vector_x = std::atoi(data_fields[4].c_str()) / 1000.0;
          rti01_.vector_y = std::atoi(data_fields[5].c_str()) / 1000.0;
          rti01_.vector_z = std::atoi(data_fields[6].c_str()) / 1000.0;
          rti01_.bottom_track_depth = std::atoi(data_fields[7].c_str()) / 1000.0;
          rti01_.water_mass_x = std::atoi(data_fields[8].c_str()) / 1000.0;
          rti01_.water_mass_y = std::atoi(data_fields[9].c_str()) / 1000.0;
          rti01_.water_mass_z = std::atoi(data_fields[10].c_str()) / 1000.0;
          rti01_.water_mass_depth = std::atoi(data_fields[11].c_str()) / 1000.0;
          rti01_.ready = true;
         // rti01_.print();
          break;
     case 02:
          if (data_fields.size() != 15) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti02_.sample_time = std::atoi(data_fields[1].c_str()) / 100.0;
          rti02_.sample_num = std::atoi(data_fields[2].c_str());
          rti02_.temperature = std::atoi(data_fields[3].c_str()) / 100.0;
          rti02_.bottom_track_x = std::atoi(data_fields[4].c_str()) / 1000.0;
          rti02_.bottom_track_y = std::atoi(data_fields[5].c_str()) / 1000.0;
          rti02_.bottom_track_z = std::atoi(data_fields[6].c_str()) / 1000.0;
          rti02_.bottom_track_depth = std::atoi(data_fields[7].c_str()) / 1000.0;
          rti02_.water_mass_x = std::atoi(data_fields[8].c_str()) / 1000.0;
          rti02_.water_mass_y = std::atoi(data_fields[9].c_str()) / 1000.0;
          rti02_.water_mass_z = std::atoi(data_fields[10].c_str()) / 1000.0;
          rti02_.water_mass_depth = std::atoi(data_fields[11].c_str()) / 1000.0;
          rti02_.ready = true;
          break;
     case 03:
          if (data_fields.size() != 17) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti03_.sample_time = std::atoi(data_fields[1].c_str()) / 100.0;
          rti03_.sample_num = std::atoi(data_fields[2].c_str());
          rti03_.temperature = std::atoi(data_fields[3].c_str()) / 100.0;
          rti03_.bottom_track_x = std::atoi(data_fields[4].c_str()) / 1000.0;
          rti03_.bottom_track_y = std::atoi(data_fields[5].c_str()) / 1000.0;
          rti03_.bottom_track_z = std::atoi(data_fields[6].c_str()) / 1000.0;
          rti03_.bottom_track_q = std::atoi(data_fields[7].c_str()) / 1000.0;
          rti03_.bottom_track_depth = std::atoi(data_fields[8].c_str()) / 1000.0;
          rti03_.water_mass_x = std::atoi(data_fields[9].c_str()) / 1000.0;
          rti03_.water_mass_y = std::atoi(data_fields[10].c_str()) / 1000.0;
          rti03_.water_mass_z = std::atoi(data_fields[11].c_str()) / 1000.0;
          rti03_.water_mass_q = std::atoi(data_fields[12].c_str()) / 1000.0;
          rti03_.water_mass_depth = std::atoi(data_fields[13].c_str()) / 1000.0;
          rti03_.ready = true;
          break;
     case 30:
          if (data_fields.size() != 6) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti30_.heading = std::atof(data_fields[1].c_str());
          rti30_.pitch = std::atof(data_fields[2].c_str());
          rti30_.roll = std::atof(data_fields[3].c_str());
          rti30_.ready = true;
          break;
     case 31:
          if (data_fields.size() != 6) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti31_.heading = std::atof(data_fields[1].c_str());
          rti31_.pitch = std::atof(data_fields[2].c_str());
          rti31_.roll = std::atof(data_fields[3].c_str());
          rti31_.ready = true;
          break;
     case 32:
          if (data_fields.size() != 8) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti32_.vector_x = std::atof(data_fields[1].c_str()); //heading
          rti32_.vector_y = std::atof(data_fields[2].c_str()); //pitch
          rti32_.vector_z = std::atof(data_fields[3].c_str()); //roll
          rti33_.pressure = std::atof(data_fields[4].c_str());
          rti33_.temperature = std::atof(data_fields[5].c_str());
          rti32_.ready = true;
          break;
     case 33:
          if (data_fields.size() != 8) {
               std::cout << "Number of data fields error: " << str << std::endl;
               return;
          }
          rti33_.heading = std::atof(data_fields[1].c_str());
          rti33_.pitch = std::atof(data_fields[2].c_str());
          rti33_.roll = std::atof(data_fields[3].c_str());
          rti33_.pressure = std::atof(data_fields[4].c_str());
          rti33_.temperature = std::atof(data_fields[5].c_str());
          rti33_.ready = true;
          break;
    }


}

double VideoRayComm::heading()
{
     return heading_;
}

double VideoRayComm::depth()
{
     return depth_;
}

double VideoRayComm::roll()
{
     return roll_;
}

double VideoRayComm::pitch()
{
     return pitch_;
}

double VideoRayComm::rov_voltage()
{
     return rov_voltage_;
}

double VideoRayComm::water_temperature()
{
     return water_temperature_;
}

double VideoRayComm::humidity()
{
     return humidity_;
}

double VideoRayComm::internal_temperature()
{
     return internal_temperature_;
}

double VideoRayComm::water_ingress()
{
     return water_ingress_;
}

double VideoRayComm::yaw_accel()
{
     return yaw_accel_;
}

double VideoRayComm::pitch_accel()
{
     return pitch_accel_;
}

double VideoRayComm::roll_accel()
{
     return roll_accel_;
}

double VideoRayComm::surge_accel()
{
     return surge_accel_;
}

double VideoRayComm::sway_accel()
{
     return sway_accel_;
}

double VideoRayComm::heave_accel()
{
     return heave_accel_;
}

