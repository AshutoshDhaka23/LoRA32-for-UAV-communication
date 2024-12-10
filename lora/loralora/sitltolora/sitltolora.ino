#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include "mavlink/common/mavlink.h"

#define RESET 15 
SoftwareSerial mySerial(4, 5); // RX, TX !! labels on relay board is swapped !!

rn2xx3 myLora(mySerial);  // Create an instance of the rn2xx3 library using SoftwareSerial

void setup() {
  // Initialize serial for debugging output
  pinMode(2, OUTPUT);
  led_on();

  Serial.begin(38400);
  
  // Initialize SoftwareSerial at the default baud rate (57600) for LoRa module
  mySerial.begin(57600);

  delay(1000);

  initialize_radio();
  led_off();
  delay(2000);
}

void initialize_radio()
{
  //reset RN2xx3
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  mySerial.flush();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  // join_result = myLora.initABP("02017201", "8D7FFEF938589D95AAD928C2E2E7E48F", "AE17E567AECC8787F749A62F5541D522");

  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = myLora.initOTAA("BE7A000000001465", "5AB9EC20C83515D73EA5C58B49003B6F");

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined cibicom");

}

void loop() {
  mavlink_message_t receivedMessage;  // MAVLink message container
  mavlink_status_t mavStatus;
  static unsigned long lastTransmissionTime = 0;
  const unsigned long transmissionInterval = 5000; // 5 seconds

  int32_t lat, lon, alt;

  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    sscanf(message.c_str(), "%d %d %d", &lat, &lon, &alt); // Extract values from the string

    Serial.print(lat); // Convert to decimal degrees
    Serial.print(lon); // Convert to decimal degrees
    Serial.print(alt); // Convert to meters

    uint8_t payload[12];
    memcpy(&payload[0], &lat, 4);
    memcpy(&payload[4], &lon, 4);
    memcpy(&payload[8], &alt, 4);

    TX_RETURN_TYPE result = myLora.txBytes(payload, sizeof(payload));
          if (result == TX_SUCCESS) {
            Serial.println("GPS data sent over LoRa successfully.");
            blink_led(2);  // Blink LED twice to indicate successful transmission
          } else {
            Serial.println("Failed to send GPS data over LoRa.");
          }
    /*if (mavlink_parse_char(MAVLINK_COMM_0, incomingByte, &receivedMessage, &mavStatus)) {
      // Handle GPS_RAW_INT MAVLink messages
      if (receivedMessage.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        unsigned long currentTime = millis();
        if (currentTime - lastTransmissionTime >= transmissionInterval) {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&receivedMessage, &gps);

          // Prepare optimized payload
          int32_t lat = gps.lat; // Latitude in 1E7 degrees
          int32_t lon = gps.lon; // Longitude in 1E7 degrees
          int32_t alt = gps.alt; // Altitude in mm

          // Create binary payload (12 bytes: 4 bytes each for lat, lon, alt)
          uint8_t payload[12];
          memcpy(&payload[0], &lat, 4);
          memcpy(&payload[4], &lon, 4);
          memcpy(&payload[8], &alt, 4);

          // Send GPS data over LoRa
          TX_RETURN_TYPE result = myLora.txBytes(payload, sizeof(payload));
          if (result == TX_SUCCESS) {
            Serial.println("GPS data sent over LoRa successfully.");
            blink_led(2);  // Blink LED twice to indicate successful transmission
          } else {
            Serial.println("Failed to send GPS data over LoRa.");
          }

          lastTransmissionTime = currentTime;  // Update lastTransmissionTime
        }
      }
    }*/
  }
}

void led_on()
{
  digitalWrite(2, 1);
}

void led_off()
{
  digitalWrite(2, 0);
}

void blink_led(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(2, HIGH);  // LED on
    delay(200);  // LED on for 200ms
    digitalWrite(2, LOW);   // LED off
    delay(200);  // LED off for 200ms
  }
}
