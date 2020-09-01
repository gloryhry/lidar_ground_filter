#include "ground_filter.h"

using namespace std;

double grid_hdifference_th_r = 0.1;

void GroundFilter::ground_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pc,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr not_ground_pc)
{
    ground_pc->clear();
    not_ground_pc->clear();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PassThrough<pcl::PointXYZI> ptfilter;
    ptfilter.setInputCloud(input_cloud);
    ptfilter.setFilterFieldName("z");
    ptfilter.setFilterLimits(-1000.0, -2.0);
    ptfilter.setFilterLimitsNegative(false);
    ptfilter.filter(*ground_pc);
    ptfilter.setFilterLimitsNegative(true);
    ptfilter.filter(*cloud);

    // std::cout << cloud->size()<<std::endl;

    // grid
    double grid_size_c = 0.25;
    double grid_size_r = 0.4;
    int column = 160 / grid_size_c + 2;
    int row = 100 / grid_size_r + 2;
    GroundFilter grid[column][row];
    for (int i = 0; i < cloud->points.size(); i++)
    {
        int m = floor(cloud->points[i].x / grid_size_r) + 2;
        int n = floor(cloud->points[i].y / grid_size_c) + column / 2;
        if (n < column && n >= 0 && m < row && m >= 0)
        {
            grid[n][m].cloud->points.push_back(cloud->points[i]);
        }
    }

    for (int i = 1; i < column - 1; i++)
    {
        for (int j = 1; j < row - 1; j++)
        {
            if (grid[i][j].cloud->size() < 2)
            {
                continue;
            }
            else
            {
                if (grid[i][j].cloud->size() == 2)
                {
                    GroundFilter temp_grid;
                    for (int ii = -1; ii < 2; ii++)
                    {
                        for (int jj = -1; jj < 2; jj++)
                        {
                            temp_grid.cloud->insert(temp_grid.cloud->end(), grid[i + ii][j + jj].cloud->begin(),
                                                    grid[i + ii][j + jj].cloud->end());
                        }
                    }
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*temp_grid.cloud, centroid);
                    grid[i][j].h_mean = centroid[2];
                    temp_grid.h_mean = centroid[2];

                    pcl::PointXYZI min;
                    pcl::PointXYZI max;
                    pcl::getMinMax3D(*temp_grid.cloud, min, max);
                    grid[i][j].h_min = min.z;
                    grid[i][j].h_max = max.z;
                    grid[i][j].h_diff = max.z - min.z;
                }
                else
                {
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*grid[i][j].cloud, centroid);
                    grid[i][j].h_mean = centroid[2];
                    pcl::PointXYZI min, max;
                    pcl::getMinMax3D(*grid[i][j].cloud, min, max);
                    grid[i][j].h_min = min.z;
                    grid[i][j].h_max = max.z;
                    grid[i][j].h_diff = max.z - min.z;
                }

                if (grid[i][j].h_diff < grid_hdifference_th_r)
                {
                    ground_pc->insert(ground_pc->end(), grid[i][j].cloud->begin(), grid[i][j].cloud->end());
                }
                else
                {
                    // pcl::PointCloud<pcl::PointXYZI>::Ptr point_i(new pcl::PointCloud<pcl::PointXYZI>());
                    // pcl::PointCloud<pcl::PointXYZI>::Ptr point_j(new pcl::PointCloud<pcl::PointXYZI>());

                    // point_j->clear();
                    for (int k = 0; k < grid[i][j].cloud->size(); k++)
                    {
                        if (grid[i][j].cloud->points[k].z - grid[i][j].h_min < 0.2)
                        {
                            ground_pc->insert(ground_pc->end(), grid[i][j].cloud->points[k]);
                        }
                        else
                        {
                            not_ground_pc->insert(not_ground_pc->end(), grid[i][j].cloud->points[k]);
                        }
                    }
                }
            }
        }
    }
}
