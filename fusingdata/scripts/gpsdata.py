#!/usr/bin/env python3.10

import rospy
from sensor_msgs.msg import NavSatFix
import serial

def parse_nmea(sentence):
    data = sentence.split(',')
    if data[0] == '$GNGGA' and len(data) >= 14:  # Check for complete data
        try:
            lat = float(data[2])
            lon = float(data[4])
            lat_dir = data[3]
            lon_dir = data[5]
            if lat_dir == 'S':
                lat = -lat
            if lon_dir == 'W':
                lon = -lon
            return lat, lon
        except ValueError:
            return None, None
    else:
        return None, None

def gps_reader():
    rospy.init_node('gps_reader', anonymous=True)
    pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)
    serial_port = serial.Serial('/dev/ttyUSB1', 38400)  # Update with your GPS device serial port and baudrate

    while not rospy.is_shutdown():
        try:
            data = serial_port.readline().decode('utf-8', errors='replace').strip()
            lat, lon = parse_nmea(data)
            if lat is not None and lon is not None:
                fix = NavSatFix()
                fix.latitude = lat
                fix.longitude = lon
                pub.publish(fix)
                rospy.loginfo("Published GPS data - Latitude: {}, Longitude: {}".format(lat, lon))
        except Exception as e:
            rospy.logerr("Error reading serial data: {}".format(e))

if __name__ == '__main__':
    try:
        print("started")
        gps_reader()
        print("after gps_reader")
    except rospy.ROSInterruptException:
        pass
