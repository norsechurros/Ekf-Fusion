#!/usr/bin/env python3.10

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
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
    pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)  # Adjust the rate as needed
    serial_port = serial.Serial('/dev/ttyACM0', 38400)  # Update with your GPS device serial port and baudrate

    while not rospy.is_shutdown():
        try:
            data = serial_port.readline().decode('utf-8', errors='replace').strip()
            lat, lon = parse_nmea(data)
            if lat is not None and lon is not None:
                fix = NavSatFix()
                fix.header = Header()
                fix.header.stamp = rospy.Time.now()
                fix.header.frame_id = "gps"
                fix.status = NavSatStatus()
                fix.status.status = 0
                fix.status.service = 0
                fix.latitude = lat
                fix.longitude = lon
                fix.altitude = 0.0                  # Update with actual altitude if available
                fix.position_covariance = [0.0] * 9  # Update with actual covariance if available
                fix.position_covariance = [9, 0, 0, 
                                           0, 9, 0, 
                                           0, 0, 9]
                # fix.position_covariance_type = 0        # 0 - Unknown, 1 - Approximated, 2 - Only Diagonal Known
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                pub.publish(fix)
                rospy.loginfo("Published GPS data - Latitude: {}, Longitude: {}".format(lat, lon))
        except Exception as e:
            rospy.logerr("Error reading serial data: {}".format(e))
        rate.sleep()

if __name__ == '__main__':
    try:
        print("started")
        gps_reader()
        print("after gps_reader")
    except rospy.ROSInterruptException:
        pass
