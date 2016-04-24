#include "tritech_micron/tritech_micron.h"
#include <iostream>
#include <vector>

using namespace tritech;

// ######################################################################
TritechMicron::TritechMicron()
{
    resetMessage();
}

// ######################################################################
TritechMicron::~TritechMicron()
{
    itsRunning = false;
    if(itsSerialThread.joinable())
        itsSerialThread.join();
}

// ######################################################################
bool TritechMicron::connect(std::string const& devName)
{
    std::cout << "Configuring" << std::endl;
    itsSerial.configure(
        /*dev*/      devName.c_str(),
        /*speed*/    115200,
        /*format*/   "8N1",
        /*flowSoft*/ false,
        /*flowHard*/ false,
        /*timeout*/  0);
    itsSerial.perror();
    std::cout << std::endl;
    itsSerial.setBlocking(true);
    std::cout << "Configured" << std::endl;

    std::cout << "Connecting" << std::endl;
    itsSerial.connect();
    itsSerial.perror();
    std::cout << std::endl;
    std::cout << "Connected" << std::endl;

    return itsSerial.isSerialOk();
}

void TritechMicron::configure(uint16_t nBins, float range, float VOS, uint8_t angleStep, uint16_t leftLimit, uint16_t rightLimit, bool cont)
{
    mtHeadCommandMsg sMsg = mtHeadCommandMsg(nBins, range, VOS, angleStep, leftLimit, rightLimit, cont);
    std::vector<uint8_t> msg = sMsg.construct();
    itsSerial.write(&msg[0], msg.size());
}

void TritechMicron::start()
{
    itsRunning = true;
    itsSerialThread = std::thread([this]() {
        serialThreadMethod();
    });
    // Send initial mtSendData command
    itsSerial.write(&mtSendDataMsg[0], mtSendDataMsg.size());
    std::cout << "Start receiving sonarline ..." << std::endl;
}

// ######################################################################
void TritechMicron::serialThreadMethod()
{
    while(itsRunning) {
        uint8_t byte;
        int bytesRead = itsSerial.read(&byte, 1);
        if(bytesRead == 1) processByte(byte);
        else {
            usleep(10000);
        }
    }
}

// ######################################################################
void TritechMicron::resetMessage()
{
    itsMsg = Message();
    itsState = WaitingForAt;
    itsRawMsg.clear();
}

// ######################################################################
void TritechMicron::processByte(uint8_t byte)
{

    if(itsState == WaitingForAt)
        if(byte == '@') {
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

    if(itsState == ReadingHeader) {
        // Ignore the 'Hex Length' section
        if(itsRawMsg.size() < 7) return;

        if(itsRawMsg.size() == 7) {
            itsMsg.binLength  = uint16_t(byte);
            return;
        }
        if(itsRawMsg.size() == 8) {
            itsMsg.binLength |= uint16_t(byte) << 8;
            return;
        }
        if(itsRawMsg.size() == 9) {
            itsMsg.txNode = byte;
            return;
        }
        if(itsRawMsg.size() == 10) {
            itsMsg.rxNode = byte;
            return;
        }
        if(itsRawMsg.size() == 11) {
            itsMsg.count  = byte;
            return;
        }
        if(itsRawMsg.size() == 12) {
            itsMsg.type = MessageType(byte);
            itsState = ReadingData;
            return;
        }

        std::cerr << "Parsing error! " << __FILE__ << ":" << __LINE__ << std::endl;
        resetMessage();
        return;
    }

    if(itsState == ReadingData) {
        if(int(itsMsg.binLength - (itsRawMsg.size() - 7)) == 0) {
            if(byte == 0x0A) {
                // std::cout << "Message Finished" << std::endl;
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

void TritechMicron::processMessage(tritech::Message msg)
{
    if(msg.type == mtVersionData) {
        mtVersionDataMsg parsedMsg(msg);
        // parsedMsg.print();
    } else if(msg.type == mtAlive) {
        mtAliveMsg parsedMsg(msg);
        // parsedMsg.print();
    } else if(msg.type == mtHeadData) {
        mtHeadDataMsg parsedMsg(msg);
        // parsedMsg.print();
        scanLinePub(parsedMsg);
        itsSerial.write(&mtSendDataMsg[0], mtSendDataMsg.size());
    } else {
        std::cerr << "Unhandled Message Type: " << msg.type << std::endl;
    }
}
