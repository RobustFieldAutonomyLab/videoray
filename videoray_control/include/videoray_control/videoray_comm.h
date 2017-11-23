#ifndef VIDEORAY_COMM_VIDEORAY_COMM_H_
#define VIDEORAY_COMM_VIDEORAY_COMM_H_
/// ----------------------------------------------------------------------------
/// @file VideoRayComm.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2014-01-21 12:40:52 syllogismrxs>
///
/// @version 1.0
/// Created: 13 Aug 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The VideoRayComm class ...
/// 
/// ----------------------------------------------------------------------------

#include "videoray_control/videoray_packetizer.h"
#include <serial/serial.h>
#include <vector>

#define MANIP_CTRL_SIZE 0x8

namespace videoray_control {

struct RTI01 {
  float sample_time;
  int sample_num;
  float temperature;
  float bottom_track_x;
  float bottom_track_y;
  float bottom_track_z;
  float bottom_track_depth;
  float water_mass_x;
  float water_mass_y;
  float water_mass_z;
  float water_mass_depth;
  bool ready;
};

struct VideoRayComm {
  enum Status_t {
    Success = 0,
    Failure
  };

  VideoRayComm(std::string port);
  ~VideoRayComm();

  Status_t set_desired_heading(int heading);
  Status_t set_desired_depth(int depth);
  Status_t set_lights(int lights);
  Status_t set_vertical_thruster(int thrust);
  Status_t set_port_thruster(int thrust);
  Status_t set_starboard_thruster(int thrust);

  Status_t send_control_command();
  Status_t send_nav_data_command();
  Status_t request_status();
  Status_t request_dvl_status();
  void parse_rti_dvl_data(const std::string &str);

  double depth_;
  double heading_;
  double roll_;
  double pitch_;
  double water_temperature_;
  double humidity_;
  double internal_temperature_;
  double water_ingress_;
  double yaw_accel_;
  double pitch_accel_;
  double roll_accel_;
  double surge_accel_;
  double sway_accel_;
  double heave_accel_;

  double rov_voltage_;
  std::vector<char> dvl_buff_;
  RTI01 rti01_;

  char tx_ctrl_data[15];
  //char tx_sensor_data[7];
  char servo_ctrl_data[MANIP_CTRL_SIZE];

  Packetizer packetizer_;
  Packetizer receiver_;
  serial::Serial serial_;

  short swap_bytes(char *in, int msb, int lsb);
};
}

#endif
