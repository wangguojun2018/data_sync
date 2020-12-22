/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-12-22 10:30:21
 */
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string>

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::NavSatFix,
                                                     nav_msgs::Odometry>
    MySyncPolicy;

class Sync
{
public:
    Sync();
    virtual ~Sync();

    ros::NodeHandle nh;

    void CallBack(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage, const sensor_msgs::NavSatFixConstPtr &infix,
                   const nav_msgs::OdometryConstPtr &inodom);
    

private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> *fix_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;
    int seq_;
    string velodyne_dir_;
    string image_dir_;
    string odom_dir_;
    string cloud_topic_;
    string image_topic_;
    string fix_topic_;
    string odom_topic_;
};