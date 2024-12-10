from pymavlink import mavutil
import serial
import time

# Serial port setup
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Replace with your serial port
    baudrate=34800,       # Set baud rate
    timeout=1             # Timeout for read operations (optional)
)

# MAVLink connection setup
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat from the MAVLink system
the_connection.wait_heartbeat()
print(f"Heartbeat received from system {the_connection.target_system}, component {the_connection.target_component}")

try:
    while True:
        # Receive the GPS data (GLOBAL_POSITION_INT message)
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        if msg:
            # Access lat, lon, and alt from the message
            lat = msg.lat  # Latitude in 1E7 degrees
            lon = msg.lon  # Longitude in 1E7 degrees
            alt = msg.alt  # Altitude in mm

            # Format the data into a string
            formatted_data = f"{lat} {lon} {alt}\n"

            # Print formatted data for debugging
            print(f"Formatted data: {formatted_data.strip()}")

            if ser.isOpen():
                # Send the formatted data over serial
                ser.write(formatted_data.encode('utf-8'))
                print(f"Sent: {formatted_data.strip()}")

            # Wait before sending the next message
            time.sleep(5)

except KeyboardInterrupt:
    print("\nExiting program...")

# Close the serial connection
finally:
    ser.close()
    print("Serial port closed")
