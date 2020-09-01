#ifndef _INCLUDE_GROUNDFILTER_HPP_
#define _INCLUDE_GROUNDFILTER_HPP_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
// #include <pcl/segmentation/progressive_morphological_filter.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

// #include <Eigen/Dense>
// #include <Eigen/Eigen>
// #include <Eigen/Core>

using namespace std;

class GroundFilter //将一个栅格定义为一个类对象
{
public:
    double h_mean;
    double h_min;
    double h_max;
    double h_diff;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZI>};

public:
    GroundFilter() {}
    ~GroundFilter() {}

    void ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_pc);
};

#endif
