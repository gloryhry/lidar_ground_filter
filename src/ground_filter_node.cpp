#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <ground_filter.h>

using namespace std;

string pkg_loc;
ros::Publisher ground_pub;
ros::Publisher no_ground_pub;

void filter_ground(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input_msg, *pointcloud);

    GroundFilter filter;
    filter.ground_filter(pointcloud, ground_pc, no_ground_pc);

    sensor_msgs::PointCloud2 no_ground_pc2, ground_pc2;
    pcl::toROSMsg(*ground_pc, ground_pc2);
    ground_pc2.header = input_msg->header;
    ground_pub.publish(ground_pc2);
    pcl::toROSMsg(*no_ground_pc, no_ground_pc2);
    no_ground_pc2.header = input_msg->header;
    no_ground_pub.publish(no_ground_pc2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_filter_node");
    pkg_loc = ros::package::getPath("pcl_test");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string input_topic, no_ground_topic, ground_topic;
    private_nh.param<string>("input_topic", input_topic, "/lidar");
    private_nh.param<string>("ground_topic", ground_topic, "/ground");
    private_nh.param<string>("no_ground_topic",no_ground_topic,"/no_ground");

    ground_pub = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 1);
    no_ground_pub = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 1);

    ros::Subscriber input_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, &filter_ground);

    ros::spin();
    return 0;
}