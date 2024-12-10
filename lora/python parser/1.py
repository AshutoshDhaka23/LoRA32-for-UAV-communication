from pymavlink import mavutil
import serial

# Set up MAVLink connection to the simulation
sim_connection = mavutil.mavlink_connection('udpin:localhost:14550')  # Replace with your simulation connection details

# Set up serial connection to ESP32
# Replace '/dev/ttyUSB1' with the correct port for your ESP32
# esp_connection = serial.Serial('/dev/ttyUSB1', baudrate=57600)

# Wait for a heartbeat from the simulation to confirm connection
print("Waiting for heartbeat from simulation...")
sim_connection.wait_heartbeat()
print(f"Heartbeat received from system {sim_connection.target_system}, component {sim_connection.target_component}")

# Request GPS_RAW_INT messages at an interval of 2 seconds
message_request = sim_connection.mav.command_long_encode(
    sim_connection.target_system,
    sim_connection.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,  # Confirmation
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,  # Message ID for GPS_RAW_INT
    5000000,  # Interval in microseconds (2 seconds)
    0, 0, 0, 0, 0  # Unused parameters
)
sim_connection.mav.send(message_request)

# Confirm the message request was accepted
response = sim_connection.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Message request accepted: GPS_RAW_INT will stream every 2 seconds")
else:
    print("Message request failed")

while 1:
    mag = sim_connection.recv_match(type='GPS_RAW_IMU', blocking=True)
    print(mag)

# Loop to forward MAVLink messages from simulation to ESP32
"""try:
    while True:
        print("Waiting for GPS_RAW_INT message...")
        # Wait for a GPS_RAW_INT message from the simulation
        mavlink_message = sim_connection.recv_match(type='GPS_RAW_INT', blocking=True)
        if mavlink_message:
            print(f"Received GPS_RAW_INT: {mavlink_message}")
            
            # Convert the MAVLink message to a byte array
            message_bytes = mavlink_message.get_msgbuf()

            # Send the byte array over serial to the ESP32
            esp_connection.write(message_bytes)
            
            print("Message forwarded to ESP32")
except KeyboardInterrupt:
    print("Exiting...")
    # esp_connection.close()
"""