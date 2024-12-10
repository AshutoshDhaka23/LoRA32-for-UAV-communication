#include "mavlink/common/mavlink.h"
#include <rn2xx3.h>
#include <SoftwareSerial.h>

bool messageRequested = false;  // to request the message only once

#define RESET 15
SoftwareSerial mySerial(4, 5);

rn2xx3 myLora(mySerial);

void setup(){
  pinMode(2, OUTPUT);
  Serial.begin(38400); // We receive messages from Mavlink at this baud rate. 

  mySerial.begin(57600); // we transmit messages from esp8266 to rn2483 using this baud rate. 

  delay(1000);

  initialize_radio(); 
  delay(1000);
}

void initialize_radio(){
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  delay(100);

  mySerial.flush();

  String hweui = myLora.hweui();
  while (hweui.length() !=16){
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    Serial.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }
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

void sendLoraMsg(){
  mavlink_message_t msg;
  mavlink_status_t status;

  if(Serial.available()>0){

    uint8_t c = Serial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if(msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT){
        mavlink_gps_raw_int_t gps_raw;
        mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);

        int32_t lat = gps_raw.lat;
        int32_t lon = gps_raw.lon;
        int32_t alt = gps_raw.alt;

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
      }
    }
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

void loop(){
  if(!messageRequested){
    sendRequestMessageInterval(1, 1, 24, 5000000);
    Serial.println("message requested");
    delay(2000);
    messageRequested = true;
  }
  sendLoraMsg();
  Serial.println("Message sent now we wait");
  delay(5000);
}