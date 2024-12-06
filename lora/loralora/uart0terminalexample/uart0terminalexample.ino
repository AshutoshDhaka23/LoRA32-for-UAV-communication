#include "mavlink/common/mavlink.h"  // MAVLink library
#include "rn2xx3.h"          // RN2483 LoRa library

// LED and timing constants
const int ledPin = 2;             // LED pin (GPIO 2)
unsigned long lastHeartbeatTime = 0;  // Last heartbeat message time
bool ledState = false;            // LED state

// LoRa credentials
const String appEUI = "BE7A000000001465";   // Replace with your App EUI
const String appKey = "5AB9EC20C83515D73EA5C58B49003B6F";   // Replace with your App Key

// LoRa RN2483 setup
HardwareSerial loraSerial(1);     // LoRa module on Serial1
rn2xx3 loraModule(loraSerial);    // LoRa RN2483 module instance

void setup() {
  // LED setup
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Debug Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP32 is ready. Listening for MAVLink heartbeat packets...");

  // LoRa module initialization
  loraSerial.begin(57600);  // LoRa module baud rate
  Serial.println("Initializing RN2483 module...");
  
  // Auto-baud LoRa module
  loraModule.autobaud();
  if (!loraModule.initOTAA(appEUI, appKey)) {
    Serial.println("Failed to join LoRaWAN network. Check credentials.");
    while (1); // Halt execution
  }
  Serial.println("Successfully joined LoRaWAN network!");
}

void loop() {
  mavlink_message_t receivedMessage;  // MAVLink message container
  mavlink_status_t mavStatus;        // MAVLink parsing status

  // Check for MAVLink data availability
  while (Serial.available()) {
    uint8_t incomingByte = Serial.read();  // Read incoming byte

    // Parse MAVLink byte stream
    if (mavlink_parse_char(MAVLINK_COMM_0, incomingByte, &receivedMessage, &mavStatus)) {
      // Handle heartbeat messages
      if (receivedMessage.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(&receivedMessage, &heartbeat);

        // Display heartbeat details
        Serial.println("Heartbeat Message Received:");
        Serial.print("    Type: ");
        Serial.println(heartbeat.type);
        Serial.print("    Autopilot: ");
        Serial.println(heartbeat.autopilot);
        Serial.print("    System Status: ");
        Serial.println(heartbeat.system_status);

        // Send data every 3 seconds
        unsigned long currentTime = millis();
        if (currentTime - lastHeartbeatTime >= 3000) {
          // Format payload for LoRa
          String payload = "HB:" + String(heartbeat.type) +
                           ",AP:" + String(heartbeat.autopilot) +
                           ",SS:" + String(heartbeat.system_status);

          // Send heartbeat data over LoRa
          TX_RETURN_TYPE result = loraModule.txUncnf(payload);
          if (result == TX_SUCCESS) {
            Serial.println("Heartbeat data sent over LoRa successfully.");
            
            // Blink LED to indicate transmission
            ledState = !ledState;
            digitalWrite(ledPin, ledState);
          } else {
            Serial.println("Failed to send heartbeat data over LoRa.");
          }

          // Update last transmission time
          lastHeartbeatTime = currentTime;
        }
      }
    }
  }
}
