#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <Arduino.h>

// Struct to represent GPS_RAW_INT packets
struct GPSRawInt {
  uint64_t time_usec;  // Timestamp
  int32_t lat;         // Latitude in 1E7 format
  int32_t lon;         // Longitude in 1E7 format
  int32_t alt;         // Altitude in millimeters
};

// Circular Buffer Class
class CircularBuffer {
  public:
    CircularBuffer();
    CircularBuffer(int size);            // Constructor to initialize the buffer
    ~CircularBuffer();                   // Destructor to free memory
    bool add(const GPSRawInt &packet);   // Add a packet to the buffer
    bool retrieve(GPSRawInt &packet);    // Retrieve a packet from the buffer
    bool isFull() const;                 // Check if the buffer is full
    bool isEmpty() const;                // Check if the buffer is empty
    int count() const;                   // Get the current number of packets in the buffer

  private:
    GPSRawInt *buffer;                   // Pointer to the buffer array
    int bufferSize;                      // Maximum size of the buffer
    int writeIndex;                      // Index to write the next packet
    int readIndex;                       // Index to read the next packet
    int packetCount;                     // Current count of packets in the buffer
};

#endif
