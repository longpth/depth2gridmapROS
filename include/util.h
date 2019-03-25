#pragma once

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl_ros/transforms.h>
#include <memory>
#include <string>

namespace depth2gridmap {

  namespace util{

    typedef struct camInfo{
      float fx;
      float fy;
      float cx;
      float cy;
      float width;
      float height;
    }CAM_INFO;

    extern std::vector<std::string> GetDirectoryFiles(const std::string& dir);
    extern pcl::PointCloud<pcl::PointXYZRGB>* createXYZRGBPointCloud ( const cv::Mat& depth_img, 
                                                                      const cv::Mat& rgb_img,
                                                                      float scaling,
                                                                      float maxDepth,
                                                                      CAM_INFO camInfo);
    extern pcl::PointCloud<pcl::PointXYZRGB>* createXZYRGBPointCloud ( const cv::Mat& depth_img, 
                                                                      const cv::Mat& rgb_img,
                                                                      float scaling,
                                                                      float maxDepth,
                                                                      CAM_INFO camInfo);
    extern void print4x4Matrix (const Eigen::Matrix4d & matrix);
    extern void updateGrid(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                          nav_msgs::OccupancyGridPtr grid, tf::Transform trans, double maxDepth);
    extern void initGrid(nav_msgs::OccupancyGridPtr grid, std::string frame_id, int width, int height, double cellResolution);

  }
}