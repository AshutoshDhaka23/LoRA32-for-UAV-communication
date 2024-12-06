#include <rn2xx3.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5); // RX, TX !! labels on relay board is swapped !!



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

  // Send break condition to initiate auto-baud detection
  sendBreakCondition();
  
  // Send 0x55 character for auto-baud synchronization
  mySerial.write(0x55);
  
  // Change the baud rate to 9600 for more stable communication
  mySerial.end();  // End current baud rate
  delay(100);
  mySerial.begin(9600);  // Start communication at 9600

  // Re-initialize LoRa at the new baud rate
  myLora.init();

  // Enable Adaptive Data Rate (ADR)
}

void sendBreakCondition() {
  // Send a break condition (holding TX line LOW) to trigger the RN2483 auto-baud detection
  mySerial.flush();  // Wait until all data is sent
  pinMode(4, OUTPUT);  // Set TX pin as an output
  digitalWrite(4, LOW);  // Set TX line LOW
  delay(10);  // Hold LOW for 10ms (long enough for a break condition)
  pinMode(4, INPUT);  // Set back to INPUT (SoftwareSerial will regain control)
}
