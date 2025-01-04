#include "MavlinkRequestor.h"

MavlinkRequestor::MavlinkRequestor(Stream& serial) : _serial(serial) {}

void MavlinkRequestor::requestGPSRawInt(uint8_t systemID, uint8_t componentID, uint16_t intervalMS) {
  // Request MAVLink message: GPS_RAW_INT (ID 24)
  sendCommandLong(systemID, componentID, MAVLINK_MSG_ID_GPS_RAW_INT, intervalMS);
}

void MavlinkRequestor::sendHeartbeat(uint8_t systemID, uint8_t componentID) {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  // Construct heartbeat message
  mavlink_msg_heartbeat_pack(systemID, componentID, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                             MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);

  // Serialize and send
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  _serial.write(buffer, len);
}

void MavlinkRequestor::sendCommandLong(uint8_t systemID, uint8_t componentID, uint16_t messageID, uint16_t intervalMS) {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  // Construct COMMAND_LONG message to request message stream
  mavlink_msg_command_long_pack(
    systemID, componentID, &msg, 
    systemID, componentID, // Target system/component
    MAV_CMD_SET_MESSAGE_INTERVAL, // Command to set message interval
    0, // Confirmation
    messageID,                 // Param 1: Message ID
    intervalMS * 1000,         // Param 2: Interval in microseconds
    0, 0, 0, 0, 0             // Remaining params
  );

  // Serialize and send
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  _serial.write(buffer, len);
}

void MavlinkRequestor::periodicGPSRawIntRequest(uint8_t systemID, uint8_t componentID, uint16_t intervalMS, unsigned long periodMS) {
  unsigned long currentTime = millis();
  if (currentTime - lastRequestTime >= periodMS) {
    requestGPSRawInt(systemID, componentID, intervalMS);
    Serial.println("Requested GPS_RAW_INT");
    lastRequestTime = currentTime;
  }
}