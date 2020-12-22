/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-12-22 11:08:40
 */
#include <data_sync/sync.h>
#include <fstream>
#include <boost/bind/bind.hpp>

#include <sensor_msgs/image_encodings.h> //图像编码格式
#include <opencv2/imgproc/imgproc.hpp>   //图像处理
#include <opencv2/highgui/highgui.hpp>   //opencv GUI

// using namespace std;
Sync::Sync()
{
     ros::param::get("~cloud_topic", cloud_topic_);
     ROS_INFO_STREAM("获取点云话题 "<<cloud_topic_);

      ros::param::get("~image_topic", image_topic_);
     ROS_INFO_STREAM("获取图像话题 "<<image_topic_);

      ros::param::get("~fix_topic", fix_topic_);
     ROS_INFO_STREAM("获取fix话题 "<<fix_topic_);

      ros::param::get("~odom_topic", odom_topic_);
     ROS_INFO_STREAM("获取odom话题 "<<odom_topic_);

      ros::param::get("~cloud_dir", velodyne_dir_);
     ROS_INFO_STREAM("点云目录 "<<velodyne_dir_);

      ros::param::get("~image_dir", image_dir_);
     ROS_INFO_STREAM("图像目录 "<<image_dir_);

      ros::param::get("~odom_dir", odom_dir_);
     ROS_INFO_STREAM("odom目录 "<<odom_dir_);

     cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, cloud_topic_, 1000);
     image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, image_topic_, 1000);
     fix_sub_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, fix_topic_, 1000);
     odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic_, 1000);
     
     sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), *cloud_sub_, *image_sub_, *fix_sub_, *odom_sub_);
     sync_->setAgePenalty(3);
     sync_->setMaxIntervalDuration(ros::Duration(0.12));
     seq_ = 0;
     sync_->registerCallback(boost::bind(&Sync::CallBack, this, _1, _2, _3, _4));

     ROS_INFO_STREAM("数据同步器初始化成功...");
}

Sync::~Sync()
{
    
}
void Sync::CallBack(const sensor_msgs::PointCloud2ConstPtr &incloud, const sensor_msgs::ImageConstPtr &inimage, const sensor_msgs::NavSatFixConstPtr &infix,
                       const nav_msgs::OdometryConstPtr &inodom)
{
     ROS_INFO_STREAM("同步回调...");
     ROS_INFO_STREAM("点云时间戳： "<<incloud->header.stamp);
     ROS_INFO_STREAM("图像时间戳： "<<inimage->header.stamp);
     ROS_INFO_STREAM("Fix时间戳： "<<infix->header.stamp);
     ROS_INFO_STREAM("Odom时间戳： "<<inodom->header.stamp);

     //转换点云
     pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);
     pcl::fromROSMsg(*incloud, *outcloud);
     std::ostringstream oss;
     oss << std::setfill('0') << std::setw(6) << seq_;
     pcl::io::savePCDFile(velodyne_dir_ + oss.str() + ".pcd", *outcloud,true);

     //写时间戳
     ofstream velodyne_stamp((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          velodyne_stamp.open((velodyne_dir_ + "velodyne_stamp.txt").c_str(), ios::out);
     }
     velodyne_stamp << incloud->header.stamp <<endl;
     velodyne_stamp.close();

     //转换图像
     cv_bridge::CvImagePtr cv_ptr= cv_bridge::toCvCopy(inimage, sensor_msgs::image_encodings::BGR8);
     cv::imwrite(image_dir_ + oss.str() + ".png", cv_ptr->image);
     //写时间戳
     ofstream image_stamp((image_dir_ + "image_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          image_stamp.open((image_dir_ + "image_stamp.txt").c_str(), ios::out);
     }
     image_stamp << inimage->header.stamp << endl;
     image_stamp.close();

     //转换imu
     ofstream odom_file((odom_dir_ + oss.str() + ".txt").c_str(), ios::out);
     //经纬度信息
     odom_file << infix->latitude << infix->longitude << 
              infix->altitude;

     //位置朝向信息
     odom_file << inodom->pose.pose.position.x  
              << inodom->pose.pose.position.y
              <<inodom->pose.pose.position.z 
              << inodom->pose.pose.orientation.x
              << inodom->pose.pose.orientation.y 
              << inodom->pose.pose.orientation.z 
              << inodom->pose.pose.orientation.w<<endl;

     odom_file.close();

     //写时间戳
     ofstream odom_stamp((odom_dir_ + "odom_stamp.txt").c_str(), ios::app);
     if (!velodyne_stamp)
     {
          odom_stamp.open((odom_dir_ + "odom_stamp.txt").c_str(), ios::out);
     }
     odom_stamp << inodom->header.stamp << endl;
     odom_stamp.close();
}

     