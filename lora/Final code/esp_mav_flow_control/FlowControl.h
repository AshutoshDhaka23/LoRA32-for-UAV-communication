#ifndef FLOWCONTROL_H
#define FLOWCONTROL_H

#include <Arduino.h>

class FlowControl {
  public:
    FlowControl(Stream &serial, int bufferSize); // Constructor
    void checkFlowControl();                    // Monitors the serial buffer and sends XON/XOFF
    void printBufferState();                    // Prints buffer status to the terminal
    bool isFlowPaused();                        // Returns the state of flow control

  private:
    Stream &_serial;       // Reference to the serial object
    int _bufferSize;       // Maximum size of the buffer
    bool _flowPaused;      // Indicates whether flow is paused (XOFF sent)
};

#endif
