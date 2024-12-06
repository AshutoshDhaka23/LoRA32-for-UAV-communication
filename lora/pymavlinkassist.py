from pymavlink import mavutil
import zlib
import serial 
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Connect to the ESP32
#esp_serial = serial.Serial('/dev/ttyUSB0', baudrate=38400, timeout=1)  # Adjust to match your ESP32's serial port and baudrate

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

while 1:
    msg = the_connection.recv_match(type='GPS_RAW_INT', blocking=True)
    if(msg):
        gps_data = {
            'time_usec': msg.time_usec,
            'lat': msg.lat,
            'lon': msg.lon,
            'alt': msg.alt,
            'eph': msg.eph,
            'epv': msg.epv,
            'vel': msg.vel,
            'cog': msg.cog,
            'fix_type': msg.fix_type,
            'satellites_visible': msg.satellites_visible
        }

        data_str = f"{gps_data['lat']},{gps_data['lon']},{gps_data['alt']},{gps_data['fix_type']},{gps_data['satellites_visible']}"
        compressed_data = zlib.compress(data_str.encode('utf-8'))

        # esp_serial.write(compressed_data)
        #print(f"Sent to ESP32: {compressed_data}")

        # For debugging purposes, print uncompressed GPS data
        print("GPS Data:", gps_data)
        
    time.sleep(4.5)  # Adjust to match the desired frequency of data transmission