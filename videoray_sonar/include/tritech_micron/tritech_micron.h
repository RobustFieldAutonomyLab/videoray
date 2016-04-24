#ifndef TRITECHMICRON_TRITECHMICRON_H
#define TRITECHMICRON_TRITECHMICRON_H

#include "tritech_micron/Constants.h"
#include "tritech_micron/MessageTypes.h"
#include "serial/Serial.h"
#include <vector>
#include <stdint.h>
#include <thread>
#include <future>
#include <boost/function.hpp>

class TritechMicron
{
  public:
    typedef boost::function<void(const tritech::mtHeadDataMsg &)> UseScanLineFunc;
    UseScanLineFunc scanLinePub;
    enum StateType 
    {
      WaitingForAt  = 0, //!< Waiting for an @ to appear
      ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data     
      ReadingData   = 2, //!< The header has been read, now we're just reading the data
    };

    TritechMicron();
    ~TritechMicron();

    bool connect(std::string const& devName);
    void configure(uint16_t bBins, float range, float VOS, uint8_t angleStep, uint16_t leftLimit, uint16_t rightLimit, bool cont);
    void start();

    void processByte(uint8_t byte);
    void processMessage(tritech::Message msg);
    void resetMessage();

  //private:
    void serialThreadMethod();
    std::thread itsSerialThread;
    StateType itsState;
    std::vector<uint8_t> itsRawMsg; //!< The current message begin read in
    tritech::Message itsMsg;

    Serial itsSerial;

    bool itsRunning;

};

#endif // TRITECHMICRON_TRITECHMICRON_H

