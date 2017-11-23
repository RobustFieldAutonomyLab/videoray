#include "tritech_micron/tritech_micron.h"

using namespace tritech_micron;

TritechMicron::TritechMicron(ros::NodeHandle nh) : nh_(nh) {
  std::string scanlineTopic = nh_.param("topic", std::string("sonarscan"));
  scanlinePub_ = nh_.advertise<videoray_msgs::ScanLine>(scanlineTopic, 100, false);

  std::string devName = nh_.param("dev", std::string("/dev/micron"));

  uint16_t nBins = nh_.param("nbins", 200);
  float range = nh_.param("range", 10.0);
  float VOS = nh_.param("VOS", 1500.0);
  uint8_t angleStep = nh_.param("angle_step", 32);
  uint16_t leftLimit = nh_.param("left_limit", 0);
  uint16_t rightLimit = nh_.param("right_limit", 6399);
  bool cont = nh_.param("cont", true);

  if (connect(devName))
    configure(nBins, range, VOS, angleStep, leftLimit, rightLimit, cont);

  resetMessage();
}

bool TritechMicron::connect(const std::string &devName) {
  ROS_INFO_STREAM("Configuring Tritech Micron...");
  serial_.setPort(devName);
  serial_.setBaudrate(115200);
  serial_.setBytesize(serial::eightbits);
  serial_.setParity(serial::parity_none);
  serial_.setStopbits(serial::stopbits_one);
  serial_.setFlowcontrol(serial::flowcontrol_none);
  // block
  serial::Timeout maxTimeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
  serial_.setTimeout(maxTimeout);

  serial_.open();
  if (!serial_.isOpen()) {
    ROS_ERROR_STREAM("Failed to connect to Tritech Micron serial port: " << devName);
  }

  return serial_.isOpen();
}

void TritechMicron::configure(uint16_t nBins,
                              float range,
                              float VOS,
                              uint8_t angleStep,
                              uint16_t leftLimit,
                              uint16_t rightLimit,
                              bool cont) {
  ROS_INFO_STREAM("Configuration: nBins " << nBins << " range " << range << " VOS " << VOS <<
                                          " angleStep " << int(angleStep) << " leftLimit " << leftLimit << " rightLimit " <<
                                          rightLimit << " cont " << cont);
  mtHeadCommandMsg sMsg = mtHeadCommandMsg(nBins, range, VOS, angleStep, leftLimit, rightLimit, cont);
  std::vector<uint8_t> msg = sMsg.construct();
  serial_.write(msg);
}

void TritechMicron::spin() {
  if (!serial_.isOpen())
    return;

  // Send initial mtSendData command
  serial_.write(mtSendDataMsg);
  ROS_INFO_STREAM("Start receiving scanline from Tritech Micron...");

  while (ros::ok()) {
    uint8_t byte;
    int bytesRead = serial_.read(&byte, 1);
    if (bytesRead == 1)
      processByte(byte);
    else {
      usleep(10000);
    }
  }
}

void TritechMicron::resetMessage() {
  itsMsg = Message();
  itsState = WaitingForAt;
  itsRawMsg.clear();
}

void TritechMicron::processByte(uint8_t byte) {
  if (itsState == WaitingForAt)
    if (byte == '@') {
      itsRawMsg.clear();
      // Tritech's datasheet refers to the first byte as '1' rather than '0', so let's push back a bogus byte here just
      // to make reading the datasheet easier.
      itsRawMsg.push_back(0);
      itsRawMsg.push_back(byte);
      itsState = ReadingHeader;
      itsMsg = Message();
      return;
    }

  itsRawMsg.push_back(byte);

  if (itsState == ReadingHeader) {
    // Ignore the 'Hex Length' section
    if (itsRawMsg.size() < 7) return;

    if (itsRawMsg.size() == 7) {
      itsMsg.binLength = uint16_t(byte);
      return;
    }
    if (itsRawMsg.size() == 8) {
      itsMsg.binLength |= uint16_t(byte) << 8;
      return;
    }
    if (itsRawMsg.size() == 9) {
      itsMsg.txNode = byte;
      return;
    }
    if (itsRawMsg.size() == 10) {
      itsMsg.rxNode = byte;
      return;
    }
    if (itsRawMsg.size() == 11) {
      itsMsg.count = byte;
      return;
    }
    if (itsRawMsg.size() == 12) {
      itsMsg.type = MessageType(byte);
      itsState = ReadingData;
      return;
    }

    ROS_ERROR_STREAM("Parsing scanline error!");
    resetMessage();
    return;
  }

  if (itsState == ReadingData) {
    if (int(itsMsg.binLength - (itsRawMsg.size() - 7)) == 0) {
      if (byte == 0x0A) {
//        std::cout << "Message Finished" << std::endl;
        itsMsg.data = itsRawMsg;
        processMessage(itsMsg);
        resetMessage();
      } else {
        std::cerr << "Message finished, but no LF detected!" << std::endl;
        resetMessage();
        return;
      }
    }
  }
}

void TritechMicron::processMessage(const tritech_micron::Message &msg) {
  if (msg.type == mtVersionData) {
    // mtVersionDataMsg parsedMsg(msg);
    // parsedMsg.print();
  } else if (msg.type == mtAlive) {
    // mtAliveMsg parsedMsg(msg);
    // parsedMsg.print();
  } else if (msg.type == mtHeadData) {
    mtHeadDataMsg parsedMsg(msg);
    // parsedMsg.print();
    publish(parsedMsg);
    serial_.write(mtSendDataMsg);
    rostime_ = ros::Time::now();
  } else {
    std::cerr << "Unhandled Message Type: " << msg.type << std::endl;
  }
}

void TritechMicron::publish(const mtHeadDataMsg &msg) {
  scanline_.stamp = rostime_;
  scanline_.angle = msg.bearing_degrees / 180.0 * M_PI;
  scanline_.intensities.resize(msg.scanLine.size());
  scanline_.range_max = msg.rangeScale;
  for (int i = 0; i < scanline_.intensities.size(); ++i)
    scanline_.intensities[i] = msg.scanLine[i];
  scanlinePub_.publish(scanline_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tritech_micron");
  ros::NodeHandle nh("~");

  TritechMicron sonarNode(nh);
  sonarNode.spin();
}
