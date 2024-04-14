#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher map_goal_xy_pub;
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf_listener = nullptr;

void publishUTMtoMap()
{
    double latitude, longitude;

    // Get latitude and longitude from ROS parameters
    ros::param::get("~latitude", latitude);
    ros::param::get("~longitude", longitude);

    // Convert latitude and longitude to UTM
    geographic_msgs::GeoPoint geo_point;
    geo_point.latitude = latitude;
    geo_point.longitude = longitude;

    geodesy::UTMPoint utm_point = geodesy::fromMsg(geo_point);

    // Convert UTM to geometry_msgs::PointStamped for transformation
    geometry_msgs::PointStamped utm_point_msg;
    utm_point_msg.header.stamp = ros::Time::now();
    utm_point_msg.header.frame_id = "utm";
    utm_point_msg.point.x = utm_point.easting;
    utm_point_msg.point.y = utm_point.northing;
    utm_point_msg.point.z = utm_point.altitude;

    // Transform UTM coordinates to map frame
    geometry_msgs::PointStamped map_point_msg;
    try {
        map_point_msg = tf_buffer.transform(utm_point_msg, "map", ros::Duration(1.0));
        map_goal_xy_pub.publish(map_point_msg);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to transform UTM to map frame: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_goal_to_map");
    ros::NodeHandle nh;

    // Initialize TF listener
    tf_listener = new tf2_ros::TransformListener(tf_buffer);

    // Publish Map XY topic
    map_goal_xy_pub = nh.advertise<geometry_msgs::PointStamped>("map_xy_topic", 10);

    // Wait for first transform to be available to avoid lookup errors
    ROS_INFO("Waiting for TF between 'utm' and 'map'...");
    tf_buffer.waitForTransform("utm", "map", ros::Time(0), ros::Duration(3.0));

    // Periodically publish UTM to Map conversion
    ros::Rate rate(1); // 1 Hz, adjust as needed
    while (ros::ok())
    {
        publishUTMtoMap();
        ros::spinOnce();
        rate.sleep();
    }

    // Clean up
    delete tf_listener;

    return 0;
}
