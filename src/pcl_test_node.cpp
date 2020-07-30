//
// Created by adam on 18-9-21.
//
#include <ros/ros.h>
#include "pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string lidar_topic;

    private_nh.param<std::string>("lidar_topic",lidar_topic,"/PointCloud2");

    PclTestCore core(nh,lidar_topic);
    // core.Spin();
    return 0;
}