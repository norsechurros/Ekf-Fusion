#!/usr/bin/env python3.10

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from vectornav.msg import Ins  # Assuming this is the message type published by vectornav

def vectornav_to_imu(data):
    imu_msg = Imu()

    # Fill header
    imu_msg.header = Header()
    imu_msg.header.stamp = rospy.Time.now()  # Assuming you want to use the current ROS time
    imu_msg.header.frame_id = "imu_frame"  # Change the frame_id as needed

    # Fill orientation
    imu_msg.orientation = Quaternion(data.roll, data.pitch, data.yaw, 0.0)  # Assuming roll, pitch, and yaw are in radians

    # Fill orientation covariance
    imu_msg.orientation_covariance = [0.0] * 9  # Identity matrix, indicating no uncertainty

    # Fill angular velocity
    imu_msg.angular_velocity = Vector3(data.angular_vel_x, data.angular_vel_y, data.angular_vel_z)

    # Fill angular velocity covariance
    imu_msg.angular_velocity_covariance = [0.0] * 9  # Assuming angular velocity covariance is unknown

    # Fill linear acceleration
    imu_msg.linear_acceleration = Vector3(data.linear_accel_x, data.linear_accel_y, data.linear_accel_z)

    # Fill linear acceleration covariance
    imu_msg.linear_acceleration_covariance = [0.0] * 9  # Assuming linear acceleration covariance is unknown

    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('vectornav_to_imu')

    # Subscriber to the vectornav data topic
    rospy.Subscriber('vectornav_data_topic', Ins, vectornav_to_imu)

    # Publisher for the converted IMU data
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)

    rospy.spin()
