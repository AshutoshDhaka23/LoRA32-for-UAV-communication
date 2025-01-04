#include "mavlink/common/mavlink.h"
#include <rn2xx3.h>
#include <SoftwareSerial.h>
#include <FlowControl.h>

bool messageRequested = false;
#define BUFFER_SIZE 128
#define RESET 15
SoftwareSerial mySerial(4, 5);
rn2xx3 myLora(mySerial);
unsigned long lastRequestTime = 0;

void setup() {
    pinMode(2, OUTPUT);
    Serial.begin(38400);
    mySerial.begin(57600);
    delay(1000);
    initialize_radio();
    delay(1000);
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
  Serial.println("Successfully joined TTN");

}

void sendRequestMessageInterval(uint8_t target_sysid, uint8_t target_compid, uint16_t msg_id, uint32_t interval_us) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(255, 0, &msg, target_sysid, target_compid, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval_us, 0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
}

void sendLoraMsg() {
    mavlink_message_t msg;
    mavlink_status_t status;
    while (Serial.available()) {
        uint8_t c = Serial.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps_raw;
                mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
                uint8_t payload[12];
                memcpy(&payload[0], &gps_raw.lat, 4);
                memcpy(&payload[4], &gps_raw.lon, 4);
                memcpy(&payload[8], &gps_raw.alt, 4);
                if (myLora.txBytes(payload, sizeof(payload)) == TX_SUCCESS) {
                    Serial.println("GPS data sent successfully.");
                    blink_led(2);
                } else {
                    Serial.println("Failed to send GPS data.");
                    blink_led(5);
                }
            }
        }
    }
}

void blink_led(int count) {
    for (int i = 0; i < count; i++) {
        digitalWrite(2, HIGH);
        delay(200);
        digitalWrite(2, LOW);
        delay(200);
    }
}

void loop() {
    if (!messageRequested || (millis() - lastRequestTime > 30000)) {
        sendRequestMessageInterval(1, 1, 24, 5000000);
        lastRequestTime = millis();
        messageRequested = true;
        Serial.println("Re-requested MAVLink message interval.");
    }
    sendLoraMsg();
    delay(1000);
}
