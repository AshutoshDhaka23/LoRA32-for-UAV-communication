#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include "mavlink/common/mavlink.h"

SoftwareSerial mySerial(4, 5); // RX, TX !! labels on relay board is swapped !!

rn2xx3 myLora(mySerial);  // Create an instance of the rn2xx3 library using SoftwareSerial

void setup() {
  // Initialize serial for debugging output
  Serial.begin(115200);
  
  // Initialize SoftwareSerial at the default baud rate (57600) for LoRa module
  mySerial.begin(57600);

  // Initialize LED pin and make sure it's off initially
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); // Ensure LED is initially off

  // Start LoRa module initialization at default baud rate
  myLora.init();

  // Configure OTAA settings for joining the network
  Serial.println("Trying to join TTN using OTAA");
  bool join_result = myLora.initOTAA("BE7A000000001465", "5AB9EC20C83515D73EA5C58B49003B6F");

  while (!join_result) {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); // delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");
}

void sendBreakCondition() {
  // Send a break condition (holding TX line LOW) to trigger the RN2483 auto-baud detection
  mySerial.flush();  // Wait until all data is sent
  pinMode(4, OUTPUT);  // Set TX pin as an output
  digitalWrite(4, LOW);  // Set TX line LOW
  delay(10);  // Hold LOW for 10ms (long enough for a break condition)
  pinMode(4, INPUT);  // Set back to INPUT (SoftwareSerial will regain control)
}

void loop() {
  mavlink_message_t receivedMessage;  // MAVLink message container
  mavlink_status_t mavStatus;
  static unsigned long lastTransmissionTime = 0;
  const unsigned long transmissionInterval = 3000; // 3 seconds

  while (Serial.available()) {
    uint8_t incomingByte = Serial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, incomingByte, &receivedMessage, &mavStatus)) {
      // Handle GPS_RAW_INT MAVLink messages
      if (receivedMessage.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        unsigned long currentTime = millis();
        if (currentTime - lastTransmissionTime >= transmissionInterval) {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&receivedMessage, &gps);

          // Display GPS details
          Serial.println("GPS_RAW_INT Message Received:");
          Serial.print("    Lat: ");
          Serial.println(gps.lat / 1e7, 7);
          Serial.print("    Lon: ");
          Serial.println(gps.lon / 1e7, 7);
          Serial.print("    Alt: ");
          Serial.println(gps.alt / 1e3, 3);

          // Create LoRa payload
          String payload = "GPS:Lat:" + String(gps.lat / 1e7, 7) +
                           ",Lon:" + String(gps.lon / 1e7, 7) +
                           ",Alt:" + String(gps.alt / 1e3, 3);

          // Send GPS data over LoRa
          TX_RETURN_TYPE result = myLora.tx(payload);
          if (result == TX_SUCCESS) {
            Serial.println("GPS data sent over LoRa successfully.");
            blink_led(2);  // Blink LED twice to indicate successful transmission
          } else {
            Serial.println("Failed to send GPS data over LoRa.");
          }

          lastTransmissionTime = currentTime;  // Update lastTransmissionTime
        }
      }
    }
  }
}

void blink_led(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(2, HIGH);  // LED on
    delay(200);  // LED on for 200ms
    digitalWrite(2, LOW);   // LED off
    delay(200);  // LED off for 200ms
  }
}
