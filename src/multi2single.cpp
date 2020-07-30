#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace message_filters;

static std::unique_ptr<tf::TransformListener> g_tf_listener;
ros::Publisher combine_pub;

void lidar_conbine_callback(const sensor_msgs::PointCloud2ConstPtr input_msg_1,
														const sensor_msgs::PointCloud2ConstPtr input_msg_2)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input_msg_1, *input_cloud1);
	pcl::fromROSMsg(*input_msg_2, *input_cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans(new pcl::PointCloud<pcl::PointXYZ>());
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud2, *input_cloud_trans, *g_tf_listener);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	*output_cloud = *input_cloud1 + *input_cloud_trans;
	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*output_cloud, output_msg);
	output_msg.header = input_msg_1->header;
	combine_pub.publish(output_msg);
}

void lidar_conbine_callback(const sensor_msgs::PointCloud2ConstPtr input_msg_1,
														const sensor_msgs::PointCloud2ConstPtr input_msg_2,
														const sensor_msgs::PointCloud2ConstPtr input_msg_3)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input_msg_1, *input_cloud1);
	pcl::fromROSMsg(*input_msg_2, *input_cloud2);
	pcl::fromROSMsg(*input_msg_3, *input_cloud3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans_3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud2, *input_cloud_trans_2, *g_tf_listener);
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud3, *input_cloud_trans_3, *g_tf_listener);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	*output_cloud = *input_cloud1 + *input_cloud_trans_2;
	*output_cloud = *output_cloud + *input_cloud_trans_3;
	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*output_cloud, output_msg);
	output_msg.header = input_msg_1->header;
	combine_pub.publish(output_msg);
}

void lidar_conbine_callback(const sensor_msgs::PointCloud2ConstPtr input_msg_1,
														const sensor_msgs::PointCloud2ConstPtr input_msg_2,
														const sensor_msgs::PointCloud2ConstPtr input_msg_3,
														const sensor_msgs::PointCloud2ConstPtr input_msg_4)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud4(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*input_msg_1, *input_cloud1);
	pcl::fromROSMsg(*input_msg_2, *input_cloud2);
	pcl::fromROSMsg(*input_msg_3, *input_cloud3);
	pcl::fromROSMsg(*input_msg_4, *input_cloud4);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans_3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_trans_4(new pcl::PointCloud<pcl::PointXYZ>());
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud2, *input_cloud_trans_2, *g_tf_listener);
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud3, *input_cloud_trans_3, *g_tf_listener);
	pcl_ros::transformPointCloud(input_msg_1->header.frame_id, *input_cloud4, *input_cloud_trans_4, *g_tf_listener);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	*output_cloud = *input_cloud1 + *input_cloud_trans_2;
	*output_cloud = *output_cloud + *input_cloud_trans_3;
	*output_cloud = *output_cloud + *input_cloud_trans_4;
	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*output_cloud, output_msg);
	output_msg.header = input_msg_1->header;
	combine_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "multi2single");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	g_tf_listener.reset(new tf::TransformListener());

	int num_lidar;
	string input_lidar_topic_1, input_lidar_topic_2, input_lidar_topic_3, input_lidar_topic_4;
	string output_lidar_topic;
	private_nh.param<int>("input_lidar_num", num_lidar, 1);
	private_nh.param<string>("input_lidar_1", input_lidar_topic_1, "/");
	private_nh.param<string>("input_lidar_2", input_lidar_topic_2, "/");
	private_nh.param<string>("input_lidar_3", input_lidar_topic_3, "/");
	private_nh.param<string>("input_lidar_4", input_lidar_topic_4, "/");
	private_nh.param<string>("output_lidar_topic", output_lidar_topic, "/combine_lidar");

	combine_pub = nh.advertise<sensor_msgs::PointCloud2>(output_lidar_topic, 1);

	if (num_lidar == 2)
	{
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_1(nh, input_lidar_topic_1, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_2(nh, input_lidar_topic_2, 10);
		typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy_2;
		Synchronizer<MySyncPolicy_2> sync_2(MySyncPolicy_2(50), lidar_sub_1, lidar_sub_2);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_1);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_2);
		sync_2.registerCallback(boost::bind(&lidar_conbine_callback, _1, _2));
		ros::spin();
		return 0;
	}
	else if (num_lidar == 3)
	{
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_1(nh, input_lidar_topic_1, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_2(nh, input_lidar_topic_2, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_3(nh, input_lidar_topic_3, 10);
		typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
				MySyncPolicy_3;
		Synchronizer<MySyncPolicy_3> sync_3(MySyncPolicy_3(50), lidar_sub_1, lidar_sub_2, lidar_sub_3);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_1);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_2);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_3);
		sync_3.registerCallback(boost::bind(&lidar_conbine_callback, _1, _2, _3));
		ros::spin();
		return 0;
	}
	else if (num_lidar == 4)
	{
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_1(nh, input_lidar_topic_1, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_2(nh, input_lidar_topic_2, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_3(nh, input_lidar_topic_3, 10);
		message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_4(nh, input_lidar_topic_4, 10);
		typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
																					 sensor_msgs::PointCloud2>
				MySyncPolicy_4;
		Synchronizer<MySyncPolicy_4> sync_4(MySyncPolicy_4(50), lidar_sub_1, lidar_sub_2, lidar_sub_3, lidar_sub_4);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_1);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_2);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_3);
		ROS_INFO_STREAM("lidar topic required. lidar topic: " << input_lidar_topic_4);
		sync_4.registerCallback(boost::bind(&lidar_conbine_callback, _1, _2, _3, _4));
		ros::spin();
		return 0;
	}
	else
	{
		ROS_ERROR_STREAM("default input lidar number is NOT SET!");
	}
	return 0;
}