#!/usr/bin/env python3
import serial
import socket
import base64
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 460800
HOST = "macorsrtk.massdot.state.ma.us"
PORT = 10000
MOUNTPOINT = "RTCM3_NEAR"
# Replace with your actual password
USER_PASS = base64.b64encode(b'redline6k:mstrtech877').decode()

class RosNtripBridge(Node):
    def __init__(self):
        super().__init__('ntrip_bridge_node')
        
        # ROS 2 Publisher
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Serial Setup
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Connected to {SERIAL_PORT} at {BAUD_RATE}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            return

        self.last_gga = None
        
        # Start background threads for Serial and NTRIP
        threading.Thread(target=self.read_gps_loop, daemon=True).start()
        threading.Thread(target=self.run_ntrip_loop, daemon=True).start()

    def nmea_to_dec(self, value, direction):
        """Converts NMEA lat/long strings to decimal degrees."""
        if not value or not direction: return 0.0
        # Latitude is DDMM.MMMM, Longitude is DDDMM.MMMM
        if direction in ['N', 'S']:
            degrees = float(value[:2])
            minutes = float(value[2:])
        else:
            degrees = float(value[:3])
            minutes = float(value[3:])
        
        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def read_gps_loop(self):
        """Reads serial data, updates last_gga, and publishes ROS Fix msg."""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if "$GNGGA" in line:
                    self.last_gga = line.strip()
                    self.publish_fix(self.last_gga)
            except Exception as e:
                print(f"Serial Read Error: {e}")
                time.sleep(1)

    def publish_fix(self, gga_line):
        """Parses GGA string and publishes to /gps/fix."""
        parts = gga_line.split(',')
        if len(parts) < 10 or parts[2] == '': 
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        
        msg.latitude = self.nmea_to_dec(parts[2], parts[3])
        msg.longitude = self.nmea_to_dec(parts[4], parts[5])
        msg.altitude = float(parts[9]) if parts[9] else 0.0
        
        # Map NMEA Quality to ROS NavSatStatus
        quality = int(parts[6])
        if quality == 4: # RTK Fixed
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            print("FIXED", end="\r")
        elif quality == 5: # RTK Float
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            print("FLOAT", end="\r")
        else:
            msg.status.status = NavSatStatus.STATUS_FIX
            print("GPS LOCK", end="\r")
            
        self.fix_pub.publish(msg)

    def run_ntrip_loop(self):
        """Connects to MaCORS and feeds RTCM data back to GPS."""
        while rclpy.ok():
            if not self.last_gga:
                time.sleep(2)
                continue

            try:
                self.get_logger().info("Connecting to MaCORS...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10)
                s.connect((HOST, PORT))

                request = (f"GET /{MOUNTPOINT} HTTP/1.0\r\n"
                           f"User-Agent: NTRIP Client/1.0\r\n"
                           f"Authorization: Basic {USER_PASS}\r\n"
                           f"Ntrip-GGA: {self.last_gga}\r\n"
                           "Connection: close\r\n\r\n")
                s.sendall(request.encode())

                response = s.recv(1024).decode(errors='ignore')
                if "200 OK" in response or "ICY" in response:
                    self.get_logger().info("CONNECTED! Streaming RTCM to GPS...")
                    last_update_time = time.time()
                    
                    while rclpy.ok():
                        data = s.recv(2048)
                        if not data: break
                        
                        # Feed the corrections back to the GPS module
                        self.ser.write(data)
                        
                        # Periodically update the caster with our position (every 10s)
                        if time.time() - last_update_time > 10:
                            if self.last_gga:
                                s.sendall((self.last_gga + "\r\n").encode())
                                last_update_time = time.time()
                else:
                    self.get_logger().error(f"MaCORS Rejected: {response[:50]}")
            except Exception as e:
                self.get_logger().warn(f"NTRIP Connection Error: {e}")
                time.sleep(5)
            finally:
                s.close()

def main(args=None):
    rclpy.init(args=args)
    node = RosNtripBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()