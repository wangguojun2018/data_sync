/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-12-22 11:14:59
 */
#include<data_sync/sync.h>
#include<ros/ros.h>
#include<string.h>


using namespace std;

// string velodyne_dir = "/home/wangguojun/dataset/velodyne_dir/";
// string image_dir = "/home/wangguojun/dataset/image_dir/";
// string imu_dir = "/home/wangguojun/dataset/imu_dir/";
// string can_dir = "/home/wangguojun/dataset/can_dir/";

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "data_sync_node");

    Sync mycollect;

    ros::spin();

    return 0;
}