#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
from datetime import datetime                                     

class GPS_Node(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=0.5)
        #uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=0.5)
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        lines = []
        for _ in range(5):  # Read up to 5 lines
            try:
                line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                if line:  # Only add non-empty lines
                    lines.append(line)
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")
                break
        
        latitude = 0.0
        longitude = 0.0
        altitude = 0.0
        
        for line in lines:
            print(line)

            if line.startswith('$GPRMC'):
                [position_status, latitude, longitude] = process_gprmc(line)
            elif line.startswith('$GPVTG'):
                process_gpvtg(line)
            elif line.startswith('$GPGSA'):                                     
                process_gpgga(line)
            elif line.startswith('$GPGGA'):
                altitude = process_gptxt(line)
            elif line.startswith('$GPGSV'):
                process_gpgsv(line)
            elif line.startswith('$GPGLL'):
                process_gpgll(line)
            else:
                print("Unsupported NMEA sentence!!!")
        if latitude == None:
            latitude = 0.0
        if longitude == None:
            longitude = 0.0
        if altitude == None:
            altitude = 0.0
        msg = NavSatFix()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published GPS Data - Lat: {latitude}, Lon: {longitude}, Alt: {altitude}')

def nmea_to_decimal(nmea_str, direction):
    degrees = float(nmea_str[:2])
    minutes = float(nmea_str[2:])
    decimal = degrees + minutes / 60.0
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal
    
def process_gprmc(line):
    parts = line.split(',')
    position_status = parts[2]
    latitude = nmea_to_decimal(parts[3],parts[4]) if parts[3] and parts[4] else 0.0
    longitude = nmea_to_decimal(parts[5],parts[6]) if parts[5] and parts[6] else 0.0
    speed = parts[7]
    heading = parts[8]
    #date_str = parts[9]
    #date = datetime.strptime(date_str, '%m%d%Y').date()  
    return [position_status, latitude, longitude]


def process_gpvtg(line):
    pass

def process_gpgga(line):
    parts = line.split(',')
    altitude = float(parts[9]) if parts[9] != '' else 0.0 
    if parts[10] == 'M':
        pass
    elif parts[10] == 'KM':
        altitude *= 1000
    return altitude
def process_gptxt(line):
    pass

def process_gpgsv(line):
    pass

def process_gpgll(line):
    pass


def main(args=None):
    rclpy.init(args=args)
    node = GPS_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
