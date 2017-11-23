#include <ros/ros.h>

#include "videoray_control/videoray_comm.h"

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

namespace videoray_control {

VideoRayComm::VideoRayComm(std::string port)
  : serial_(port, 115200, serial::Timeout(serial::Timeout::max())) {
  if (!serial_.isOpen()) {
    ROS_ERROR_STREAM("Failed to connect to VideoRay serial port: " << port);
    exit(-1);
  }

  serial_.flush();

  packetizer_.set_network_id(0x01);
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
}

VideoRayComm::~VideoRayComm() {
  serial_.close();
}

VideoRayComm::Status_t VideoRayComm::set_desired_heading(int heading) {
  if (heading > 360 || heading < 0) {
    tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
    tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
  } else {
    tx_ctrl_data[AUTO_HEADING_LSB] = heading & 0x00FF;
    tx_ctrl_data[AUTO_HEADING_MSB] = (heading & 0xFF00) >> 8;
  }

  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_desired_depth(int depth) {
  if (depth < 0) {
    tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
    tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
  } else {
    tx_ctrl_data[AUTO_DEPTH_LSB] = depth & 0x00FF;
    tx_ctrl_data[AUTO_DEPTH_MSB] = (depth & 0xFF00) >> 8;
  }
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_lights(int lights) {
  tx_ctrl_data[LIGHTS_LSB] = lights;
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_vertical_thruster(int thrust) {
  tx_ctrl_data[VERT_THRUST_LSB] = thrust & 0x00FF;
  tx_ctrl_data[VERT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_port_thruster(int thrust) {
  tx_ctrl_data[PORT_THRUST_LSB] = thrust & 0x00FF;
  tx_ctrl_data[PORT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_starboard_thruster(int thrust) {
  tx_ctrl_data[STAR_THRUST_LSB] = thrust & 0x00FF;
  tx_ctrl_data[STAR_THRUST_MSB] = (thrust & 0xFF00) >> 8;
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::send_control_command() {
  char *packet;
  int bytes;

  // Generate Packet and grab reference to it
  packetizer_.set_network_id(0x01);
  packetizer_.set_flags(0x03);
  packetizer_.set_csr_addr(0x00);
  packetizer_.set_data(tx_ctrl_data, TX_CTRL_SIZE);
  bytes = packetizer_.generate_packet(&packet);

  // Send the Tx Control packet over the serial line
  serial_.write((uint8_t *) packet, bytes);

  char byte;
  Packetizer::Status_t status;
  do {
    if (serial_.read((uint8_t *) &byte, 1) == 1) {
      status = receiver_.receive_packet(byte);
    } else {
      // Did not receive a byte, break out.
      ROS_ERROR_STREAM("Error reading byte in sending control command");
      break;
    }
  } while (status == Packetizer::In_Progress);

  serial_.flush();
  return VideoRayComm::Success;
}

short VideoRayComm::swap_bytes(char *array, int msb, int lsb) {
  short temp = 0;
  temp = ((unsigned short) (array[msb] & 0xFF) << 8) | (array[lsb] & 0xFF);
  return temp;
}

VideoRayComm::Status_t VideoRayComm::send_nav_data_command() {
  char *packet;
  int bytes;

  // Navigation data vendor specific message...
  packetizer_.set_network_id(0x01);
  packetizer_.set_flags(0x5);
  packetizer_.set_csr_addr(0x0);
  packetizer_.set_data(tx_ctrl_data, 0);

  bytes = packetizer_.generate_packet(&packet);

  // Send the Tx Control packet over the serial line
  serial_.write((uint8_t *) packet, bytes);

  char byte;
  Packetizer::Status_t status;
  do {
    if (serial_.read((uint8_t *) &byte, 1) == 1) {
      status = receiver_.receive_packet(byte);
    } else {
      // Did not receive a byte, break out.
      ROS_ERROR_STREAM("Error reading byte in sending nav data command");
      break;
    }
  } while (status == Packetizer::In_Progress);

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
    ROS_ERROR_STREAM("Nav data decode error");
    serial_.flush();
  }
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::request_status() {
  char *packet;
  int bytes;

  packetizer_.set_network_id(0x01);
  packetizer_.set_flags(0x8E);
  packetizer_.set_csr_addr(0x7A);
  packetizer_.set_data(tx_ctrl_data, 0);

  bytes = packetizer_.generate_packet(&packet);

  // Send the Tx Control packet over the serial line
  serial_.write((uint8_t *) packet, bytes);

  char byte;
  Packetizer::Status_t status;
  do {
    if (serial_.read((uint8_t *) &byte, 1) == 1) {
      status = receiver_.receive_packet(byte);
    } else {
      // Did not receive a byte, break out.
      printf("Error reading byte in requesting status");
      break;
    }
  } while (status == Packetizer::In_Progress);

  if (status == Packetizer::Success) {
    bytes = receiver_.get_payload(&packet);

    //for (int x = 0 ; x < bytes ; x++) {
    //     printf("%x ", (unsigned char)packet[x]);
    //}
    //printf("\n");

    rov_voltage_ = swap_bytes(packet, VOLTAGE_12V_MSB, VOLTAGE_12V_LSB);
    water_temperature_ = swap_bytes(packet, WATER_TEMP_MSB, WATER_TEMP_LSB);
    humidity_ = swap_bytes(packet, HUMIDITY_MSB, HUMIDITY_LSB);

  } else {
    ROS_ERROR_STREAM("ROV status decode error");
    serial_.flush();
  }
  return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::request_dvl_status() {
  rti01_.ready = false;

  char *packet;
  int bytes;

  packetizer_.set_network_id(0x0A);
  packetizer_.set_flags(0x00); // Ignored
  packetizer_.set_csr_addr(0xF0);
  packetizer_.set_data(tx_ctrl_data, 0);

  bytes = packetizer_.generate_packet(&packet);

  // Send the Tx Control packet over the serial line
  serial_.write((uint8_t *) packet, bytes);

  char byte;
  Packetizer::Status_t status;
  do {
    if (serial_.read((uint8_t *) &byte, 1) == 1) {
      status = receiver_.receive_packet(byte);
    } else {
      // Did not receive a byte, break out.
      printf("Error reading byte in requesting DVL status");
      break;
    }
  } while (status == Packetizer::In_Progress);

  if (status == Packetizer::Success) {
    bytes = receiver_.get_payload(&packet);
    for (int x = 0; x < bytes; x++) {
      if (packet[x] == 16 || packet[x] == '\r' || packet[x] == 0 || packet[x] == '\n')
        continue;
      dvl_buff_.push_back(packet[x]);
    }

    while (true) {
      auto end = std::find(dvl_buff_.begin() + 1, dvl_buff_.end(), '$');
      if (end == dvl_buff_.end())
        break;

      std::string str(dvl_buff_.begin(), end);
      parse_rti_dvl_data(str);
      dvl_buff_.erase(dvl_buff_.begin(), end);
    }
  } else {
    ROS_ERROR_STREAM("DVL decode error");
    serial_.flush();
  }
  return VideoRayComm::Success;
}

void VideoRayComm::parse_rti_dvl_data(const std::string &str) {
  if (str[0] != '$') {
    ROS_ERROR_STREAM("DVL Packet doesn't start with $: " << int(str[0]) << ", " << str);
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
      rti01_.sample_time = std::atoi(data_fields[1].c_str()) / float(100.0);
      rti01_.sample_num = std::atoi(data_fields[2].c_str());
      rti01_.temperature = std::atoi(data_fields[3].c_str()) / float(100.0);
      rti01_.bottom_track_x = std::atoi(data_fields[4].c_str()) / float(1000.0);
      rti01_.bottom_track_y = std::atoi(data_fields[5].c_str()) / float(1000.0);
      rti01_.bottom_track_z = std::atoi(data_fields[6].c_str()) / float(1000.0);
      rti01_.bottom_track_depth = std::atoi(data_fields[7].c_str()) / float(1000.0);
      rti01_.water_mass_x = std::atoi(data_fields[8].c_str()) / float(1000.0);
      rti01_.water_mass_y = std::atoi(data_fields[9].c_str()) / float(1000.0);
      rti01_.water_mass_z = std::atoi(data_fields[10].c_str()) / float(1000.0);
      rti01_.water_mass_depth = std::atoi(data_fields[11].c_str()) / float(1000.0);
      rti01_.ready = true;
      break;
    default:
      // Not needed.
      break;
  }
}

}

