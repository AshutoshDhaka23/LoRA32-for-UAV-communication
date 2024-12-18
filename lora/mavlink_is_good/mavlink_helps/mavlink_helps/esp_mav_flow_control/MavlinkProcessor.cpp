#include "MavlinkProcessor.h"

MavlinkProcessor::MavlinkProcessor(CircularBuffer& buffer) : _buffer(buffer) {}

void MavlinkProcessor::processIncomingMessage(uint8_t c) {
  // Parse incoming MAVLink message
  if (mavlink_parse_char(MAVLINK_COMM_0, c, &_msg, &_status)) {
    Serial.print("Parsed MAVLink Message ID: ");
    Serial.println(_msg.msgid);
    // Process specific message types
    if (_msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
      Serial.println("Parsed GPS packet ID");
      parseGPSRawInt(_msg);
    }
  }
}

void MavlinkProcessor::parseGPSRawInt(const mavlink_message_t& msg) {
  mavlink_gps_raw_int_t gps;
  mavlink_msg_gps_raw_int_decode(&msg, &gps);

  // Create GPSRawInt object and store in circular buffer
  GPSRawInt packet = {
    gps.time_usec,
    gps.lat,
    gps.lon,
    gps.alt
  };

  if (_buffer.add(packet)) {
    Serial.println("GPS_RAW_INT Stored in Circular Buffer");
  } else {
    Serial.println("Circular Buffer Full! Packet Dropped.");
  }
}
