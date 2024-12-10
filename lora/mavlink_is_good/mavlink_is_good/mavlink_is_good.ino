#include <mavlink.h>

void setup() {
  // Initialize USB serial connection
  Serial.begin(38400); // Match SITL's MAVLink baud rate
  delay(1000); // Allow time for USB connection to establish
}

void sendRequestMessageInterval(uint8_t target_sysid, uint8_t target_compid, uint16_t msg_id, uint32_t interval_us) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Construct MAVLink COMMAND_LONG message to set message interval
  mavlink_msg_command_long_pack(
    255, // System ID of ESP32
    0,   // Component ID of ESP32
    &msg,
    target_sysid,
    target_compid,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    0, // Confirmation
    msg_id, // Message ID to request
    interval_us, // Interval in µs
    0, 0, 0, 0, 0 // Unused parameters
  );

  // Encode and send the MAVLink message
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void receiveMavlinkMessages() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // Process incoming bytes from the USB serial
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Check if the received message is GPS_RAW_INT
      if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps_raw;
        mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);

        // Print GPS data to the serial monitor (optional)
        Serial.println("GPS Data Received:");
        Serial.print("Latitude: "); Serial.println(gps_raw.lat / 1e7);
        Serial.print("Longitude: "); Serial.println(gps_raw.lon / 1e7);
        Serial.print("Altitude: "); Serial.println(gps_raw.alt / 1000.0);
      }
    }
  }
}

void loop() {
  // Request GPS_RAW_INT message at 1 Hz
  sendRequestMessageInterval(1, 1, 24, 1000000);
  
  // Handle incoming MAVLink messages
  receiveMavlinkMessages();
  
  delay(1000); // Wait 1 second before sending the next request
}
