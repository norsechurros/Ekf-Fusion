#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>
#include <nav_msgs/Odometry.h>

ros::Publisher utm_xy_pub;
//double ox=-0.06036,oy=-22.97497;
double utm_source_easting=492818.38666, utm_source_northing=5527520-2.87518;
geodesy::UTMPoint utm_point;
geometry_msgs::PointStamped utm_xy_msg;
double pose_x,pose_y;

//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract data from the Odometry message
  //  odom_x = msg->pose.pose.position.x;
    //odom_y = msg->pose.pose.position.y;
    //geometry_msgs::PointStamped utm_xy_msg;
//}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    // Convert GPS coordinates to UTM
    geographic_msgs::GeoPoint geo_point;
    geo_point.latitude = gps_msg->latitude;
    geo_point.longitude = gps_msg->longitude;
    geo_point.altitude = gps_msg->altitude;
    //std::cout<<gps_msg->latitude<<std::endl;

    //geodesy::UTMPoint utm_point;
    utm_point = geodesy::toMsg(geo_point);

    // Publish UTM XY coordinates
    // geometry_msgs::PointStamped utm_xy_msg;
    utm_xy_msg.header = gps_msg->header;
    utm_xy_msg.header.frame_id="odom";
    utm_xy_msg.point.x = utm_point.easting;
    utm_xy_msg.point.y = utm_point.northing;
    utm_xy_msg.point.z = utm_point.altitude;
    pose_x=utm_xy_msg.point.y - utm_source_northing;
    pose_y=utm_xy_msg.point.x - utm_source_easting;
    pose_y*=-1;
    //std::cout<<utm_xy_msg.point.x<<" "<<utm_xy_msg.point.y<<std::endl;
    std::cout<<pose_x<<" "<<pose_y<<std::endl;
    //utm_xy_pub.publish(utm_xy_msg);
    //std::cout<<"publishing"<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpsToutm");
    ros::NodeHandle nh;

    // Subscribe to GPS topic
    ros::Subscriber gps_sub = nh.subscribe("gps", 10, gpsCallback);

    // Publish UTM XY topic
    utm_xy_pub = nh.advertise<geometry_msgs::PointStamped>("utm_xy_topic", 10);
    std::cout<<"spinning"<<std::endl;
    ros::spin();

    return 0;
}