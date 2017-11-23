#ifndef TRITECH_MICRON_TRITECH_MICRON_H
#define TRITECH_MICRON_TRITECH_MICRON_H

#include "tritech_micron/constants.h"
#include "tritech_micron/message_types.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <videoray_msgs/ScanLine.h>

namespace tritech_micron {

class TritechMicron {
 public:
  enum StateType {
    WaitingForAt = 0, //!< Waiting for an @ to appear
    ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data
    ReadingData = 2, //!< The header has been read, now we're just reading the data
  };

  TritechMicron(ros::NodeHandle nh = ros::NodeHandle("~"));
  ~TritechMicron() {
    serial_.close();
  };

  bool connect(std::string const &devName);
  void configure(uint16_t bBins,
                 float range,
                 float VOS,
                 uint8_t angleStep,
                 uint16_t leftLimit,
                 uint16_t rightLimit,
                 bool cont);
  void spin();

  void processByte(uint8_t byte);

  void processMessage(const tritech_micron::Message &msg);

  void resetMessage();

  void publish(const mtHeadDataMsg &msg);

 private:
  ros::NodeHandle nh_;
  ros::Publisher scanlinePub_;
  ros::Publisher pointcloudPub_;
  ros::Time rostime_;
  videoray_msgs::ScanLine scanline_;

  StateType itsState;
  std::vector<uint8_t> itsRawMsg; //!< The current message begin read in
  tritech_micron::Message itsMsg;

  serial::Serial serial_;
};
}

#endif // TRITECH_MICRON_TRITECH_MICRON_H

