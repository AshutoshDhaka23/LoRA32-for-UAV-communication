#include "CircularBuffer.h"

// Default constructor
CircularBuffer::CircularBuffer() {
  // Default constructor creates an empty buffer
  bufferSize = 0;
  buffer = nullptr;
  writeIndex = 0;
  readIndex = 0;
  packetCount = 0;
}
// Constructor
CircularBuffer::CircularBuffer(int size) {
  bufferSize = size;
  buffer = new GPSRawInt[bufferSize];  // Dynamically allocate memory for the buffer
  writeIndex = 0;
  readIndex = 0;
  packetCount = 0;
}

// Destructor
CircularBuffer::~CircularBuffer() {
  delete[] buffer; // Free the allocated memory
}

// Add a packet to the buffer
bool CircularBuffer::add(const GPSRawInt &packet) {
  if (isFull()) {
    Serial.println("Circular Buffer Full!");
    return false;
  }
  buffer[writeIndex] = packet;               // Store packet
  writeIndex = (writeIndex + 1) % bufferSize; // Wrap around
  packetCount++;
  return true;
}

// Retrieve a packet from the buffer
bool CircularBuffer::retrieve(GPSRawInt &packet) {
  if (isEmpty()) {
    Serial.println("Circular Buffer Empty!");
    return false;
  }
  packet = buffer[readIndex];               // Retrieve packet
  readIndex = (readIndex + 1) % bufferSize; // Wrap around
  packetCount--;
  return true;
}

// Check if the buffer is full
bool CircularBuffer::isFull() const {
  return packetCount == bufferSize;
}

// Check if the buffer is empty
bool CircularBuffer::isEmpty() const {
  return packetCount == 0;
}

// Get the current number of packets in the buffer
int CircularBuffer::count() const {
  return packetCount;
}
