#ifndef MAVLINKREQUESTOR_H
#define MAVLINKREQUESTOR_H

#include <Arduino.h>
#include "mavlink/common/mavlink.h"

class MavlinkRequestor {
  public:
    MavlinkRequestor(Stream& serial); // Constructor to initialize the serial stream
    void requestGPSRawInt(uint8_t systemID, uint8_t componentID, uint16_t intervalMS); // Request GPS_RAW_INT
    void sendHeartbeat(uint8_t systemID, uint8_t componentID); // Send a heartbeat (optional)
    void periodicGPSRawIntRequest(uint8_t systemID, uint8_t componentID, uint16_t intervalMS, unsigned long periodMS); // Request GPS_RAW_INT periodically

  private:
    Stream& _serial; // Reference to the serial stream
    void sendCommandLong(uint8_t systemID, uint8_t componentID, uint16_t messageID, uint16_t intervalMS);
    unsigned long lastRequestTime = 0; 
};

#endif
