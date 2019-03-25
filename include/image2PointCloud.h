#pragma once

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include "util.h"

namespace depth2gridmap {

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image> CameraPolicy;

class image2PointCloud
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  image2PointCloud(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~image2PointCloud();

 private:
  /*!
   * image2PointClouds and verifies the ROS parameters.
   * @return true if successful.
   */
  bool image2PointCloudParameters();

  /*!
   * Initializes node.
   */
  void initialize();

  /*!
   * Timer callback function.
   * @param timerEvent the timer event.
   */
  void timerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Publish the point cloud as a PointCloud2.
   * @return true if successful.
   */
  bool publish();

  //! Listen to camera data, construct nodes and feed the graph manager with it.
  /*! For each dataset from the camera, do some data conversion,
    *  construct a node, hand it to the graph manager and
    *  do some visualization of the result in the GUI and RVIZ.
    */
  void imageCallback (const sensor_msgs::ImageConstPtr& visual_img,
                      const sensor_msgs::ImageConstPtr& depth_img);


  /*!
   * Publish the point cloud as a PointCloud2.
   * @param translate translation vector
   * @param rot quaternion to describe a rotation
   */
  void updateCampose(tf::Quaternion& rot, tf::Vector3& translate);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud message to publish.
  sensor_msgs::PointCloud2::Ptr pointCloudMessage_;

  //! Point cloud publisher.
  ros::Publisher pointCloudPublisher_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //! Camera pose.
  ros::Publisher camPosePublisher_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

  //! Point cloud topic to be published to.
  std::string pointCloudTopic_;

  //! Fake scan topic to be published at.
  std::string fakeScanTopic_;

  //! rgb image to be subscribed at.
  std::string rgbTopic_;

  //! depth image to be subscribed at.
  std::string depthTopic_;

  //! camera info to be subscribed at.  
  std::string camInfoTopic_;

  //! gridmap to be published to. 
  std::string gridMapTopic_;

  //! cam pose to be published to. 
  std::string camPoseTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  //! scaling rate of z axis
  float zScaling_;

  //! maximum depth be kept
  float maxDepth_;

  //! maximum depth be kept for grid map
  float maxDepthGrid_;

  //! Grid map resolution
  float gridReso_;

  //! Grid width
  int gridWidth_;

  //! Grid height
  int gridHeight_;

  //! cam pose enable
  int useCamPose_;

  //! If true, continous publishing is used.
  //! If false, point cloud is only published once.
  bool isContinousPublishing_;

  //! Duration between publishing steps.
  ros::Duration updateDuration_;

  //! Point cloud file directory
  std::vector<std::string> filesDir_;

  //! RGB Image subscriber
  message_filters::Subscriber<sensor_msgs::Image> *imageRgbSub_;

  //! depth Image subscriber
  message_filters::Subscriber<sensor_msgs::Image> *depthSub_;

  //! camera synchronizer
  message_filters::Synchronizer<CameraPolicy>* camSync_;
  
  //! listener for tf topic
  tf::TransformListener* tflistener_;

  //! accumulate point cloud 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudAcc_;

  //! point cloud of current depthmap
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudCur_; 

  //! point cloud of rotated axis
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudRot_;

  //! Normalized point cloud
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals_;

  // OccupancyGrid map message
  nav_msgs::OccupancyGridPtr gridMap_;

  // Camera pose message
  geometry_msgs::PoseStampedPtr  camPose_;

  //! First point cloud flag
  bool initFlag_;

  //! Camera info
  util::CAM_INFO camInfo_;

};

} /* namespace */
