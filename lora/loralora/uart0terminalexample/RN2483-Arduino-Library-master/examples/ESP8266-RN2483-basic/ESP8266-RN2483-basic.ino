/*
 * Author: JP Meijers
 * Date: 2016-10-20
 *
 * Transmit a one byte packet via TTN. This happens as fast as possible, while still keeping to
 * the 1% duty cycle rules enforced by the RN2483's built in LoRaWAN stack. Even though this is
 * allowed by the radio regulations of the 868MHz band, the fair use policy of TTN may prohibit this.
 *
 * CHECK THE RULES BEFORE USING THIS PROGRAM!
 *
 * CHANGE ADDRESS!
 * Change the device address, network (session) key, and app (session) key to the values
 * that are registered via the TTN dashboard.
 * The appropriate line is "myLora.initABP(XXX);" or "myLora.initOTAA(XXX);"
 * When using ABP, it is advised to enable "relax frame count".
 *
 * Connect the RN2xx3 as follows:
 * RN2xx3 -- ESP8266
 * Uart TX -- GPIO4
 * Uart RX -- GPIO5
 * Reset -- GPIO15
 * Vcc -- 3.3V
 * Gnd -- Gnd
 *
 */
#include "mavlink/common/mavlink.h"
#include <rn2xx3.h>
#include <SoftwareSerial.h>

#define RESET 15
const int ledPin = 2;
unsigned long lastHeartbeatTime = 0;
bool ledState = false;
SoftwareSerial mySerial(4, 5); // RX, TX !! labels on relay board is swapped !!

//create an instance of the rn2xx3 library,
//giving the software UART as stream to use,
//and using LoRa WAN
rn2xx3 myLora(mySerial);

// the setup routine runs once when you press reset:
void setup() {

  // LED pin is GPIO2 which is the ESP8266's built in LED
  pinMode(2, OUTPUT);
  // led_on();

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  mySerial.begin(57600);

  delay(1000); //wait for the arduino ide's serial console to open

  Serial.println("Startup");

  initialize_radio();

  //transmit a startup message
  myLora.tx("TTN Mapper on ESP8266 node");

  // led_off();
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
  Serial.println("Successfully joined TTN");

}

// the loop routine runs over and over again forever:
void loop() {
  mavlink_message_t receivedMessage;  // MAVLink message container
  mavlink_status_t mavStatus;

  while(Serial.available()){
    uint8_t incomingByte = Serial.read();

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
          TX_RETURN_TYPE result = myLora.tx(payload);
          if (result == TX_SUCCESS) {
            Serial.println("Heartbeat data sent over LoRa successfully.");
            led_on()
          } else {
            Serial.println("Failed to send heartbeat data over LoRa.");
          }
  

    Serial.println("TXing");
    myLora.tx("!"); //one byte, blocking function

}

void led_on()
{
  digitalWrite(2, 1);
}

void led_off()
{
  digitalWrite(2, 0);
}
