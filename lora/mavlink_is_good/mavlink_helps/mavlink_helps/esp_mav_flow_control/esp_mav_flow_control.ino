#include "FlowControl.h"
#include "mavlink/common/mavlink.h"
#include "CircularBuffer.h"
#include "MavlinkRequestor.h"
#include "MavlinkProcessor.h"

// Set the serial buffer size
#define BUFFER_SIZE 128
FlowControl flowControl(Serial, BUFFER_SIZE);

#define CIRCULAR_BUFFER_SIZE 10 
CircularBuffer gpsBuffer(CIRCULAR_BUFFER_SIZE);

// Mavlink objects
MavlinkRequestor mavRequestor(Serial);
MavlinkProcessor mavProcessor(gpsBuffer);

void setup() {
  Serial.begin(9600); // Initialize hardware serial
  Serial.println("Starting UART + Circular buffer flow control");

  // Request GPS_RAW_INT messages at 1 Hz
  mavRequestor.requestGPSRawInt(1, 1, 1000);
}

void loop() {
  Serial.println("We start the main loop");
  flowControl.checkFlowControl();

  mavRequestor.periodicGPSRawIntRequest(1, 1, 1000, 10000);

   while (Serial.available()) {
    uint8_t c = Serial.read();
    mavProcessor.processIncomingMessage(c);
  }
  // Print the serial buffer state every second
  static unsigned long lastTransmitTime = 0;
  if (millis() - lastTransmitTime > 5000) {
    Serial.println("Fake LoRA transmission");
    transmitFromBuffer();
    lastTransmitTime = millis();
  }
  Serial.println("UART buffer state");
  flowControl.printBufferState();
  Serial.println("Local buffer state");
  Serial.println(gpsBuffer.count());
}

void transmitFromBuffer() {
  GPSRawInt packet;
  if (gpsBuffer.retrieve(packet)) {
    // Simulate LoRa transmission by "burning" the packet
    Serial.println("Transmitting GPS_RAW_INT:");
    Serial.print("  Time: "); Serial.println(packet.time_usec);
    Serial.print("  Lat: "); Serial.println(packet.lat / 1E7, 7);
    Serial.print("  Lon: "); Serial.println(packet.lon / 1E7, 7);
    Serial.print("  Alt: "); Serial.println(packet.alt / 1000.0, 3);
  } else {
    Serial.println("Circular Buffer Empty. No Packet to Transmit.");
  }
}
