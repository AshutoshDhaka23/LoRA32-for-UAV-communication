#ifndef MAVLINKPROCESSOR_H
#define MAVLINKPROCESSOR_H

#include <Arduino.h>
#include "mavlink/common/mavlink.h"
#include "CircularBuffer.h" // Include circular buffer for storage

class MavlinkProcessor {
  public:
    MavlinkProcessor(CircularBuffer& buffer); // Constructor with circular buffer reference
    void processIncomingMessage(uint8_t c);  // Process a single incoming byte
    void parseGPSRawInt(const mavlink_message_t& msg); // Parse and store GPS_RAW_INT

  private:
    CircularBuffer& _buffer;           // Reference to circular buffer
    mavlink_message_t _msg;            // Message object
    mavlink_status_t _status;          // Status object
};

#endif
