#include "FlowControl.h"

// Constructor
FlowControl::FlowControl(Stream &serial, int bufferSize)
  : _serial(serial), _bufferSize(bufferSize), _flowPaused(false) {}

// Monitors the buffer and manages XON/XOFF flow control
void FlowControl::checkFlowControl() {
  int availableBytes = _serial.available();

  // Send XOFF if buffer exceeds high threshold
  if (!_flowPaused && availableBytes > _bufferSize * 0.8) {
    _serial.write(0x13); // XOFF
    _flowPaused = true;
    Serial.println("XOFF Sent: Pausing Transmission");
  }

  // Send XON if buffer clears below low threshold
  if (_flowPaused && availableBytes < _bufferSize * 0.2) {
    _serial.write(0x11); // XON
    _flowPaused = false;
    Serial.println("XON Sent: Resuming Transmission");
  }
}

// Prints the state of the serial buffer to the terminal
void FlowControl::printBufferState() {
  int availableBytes = _serial.available();
  Serial.print("Buffer Size: ");
  Serial.print(availableBytes);
  Serial.print(" / ");
  Serial.println(_bufferSize);
}

// Returns whether the flow is paused (XOFF sent)
bool FlowControl::isFlowPaused() {
  return _flowPaused;
}
