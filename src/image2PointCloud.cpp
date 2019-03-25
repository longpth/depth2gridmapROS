#include <math.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
//OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "image2PointCloud.h"

using namespace std;
using namespace ros;
using namespace pcl;
using namespace pcl::io;
using namespace pcl_conversions;

namespace depth2gridmap {

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type; 

#define CAMERA_POSE

image2PointCloud::image2PointCloud(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloudMessage_(new sensor_msgs::PointCloud2),
      imageRgbSub_(NULL),
      depthSub_(NULL),
      tflistener_(NULL),
      pointCloudAcc_(new pcl::PointCloud<pcl::PointXYZRGB>),
      pointCloudCur_(new pcl::PointCloud<pcl::PointXYZRGB>),
      pointCloudRot_(new pcl::PointCloud<pcl::PointXYZRGB>),
      cloudNormals_(new pcl::PointCloud<pcl::Normal>),
      gridMap_(new nav_msgs::OccupancyGrid),
      camPose_(new geometry_msgs::PoseStamped),
      initFlag_(false)
{ 
  if (!image2PointCloudParameters()) ros::requestShutdown();
  initialize();
  util::initGrid(gridMap_, pointCloudFrameId_,gridWidth_,gridHeight_,gridReso_);
}

image2PointCloud::~image2PointCloud()
{
  delete imageRgbSub_;
  delete depthSub_;
  delete tflistener_;
}

bool image2PointCloud::image2PointCloudParameters()
{
  bool allParametersimage2PointCloud = true;
  if (!nodeHandle_.getParam("topic_cloud",    pointCloudTopic_))   allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_scan",     fakeScanTopic_))     allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_rgb",      rgbTopic_))          allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_depth",    depthTopic_))        allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_caminfo",  camInfoTopic_))      allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_grid",     gridMapTopic_))      allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("topic_campose",  camPoseTopic_))      allParametersimage2PointCloud = false;
  if (!nodeHandle_.getParam("frame",          pointCloudFrameId_)) allParametersimage2PointCloud = false;

  double scaling;
  nodeHandle_.param("zScaling", scaling, 1000.0);
  zScaling_ = 1/(float)scaling;

  double maxDepth;
  nodeHandle_.param("maxDepth", maxDepth, 5.0);
  maxDepth_ = (float)maxDepth;

  double maxDepthGrid;
  nodeHandle_.param("maxDepthGrid", maxDepthGrid, 1.0);
  maxDepthGrid_ = (float)maxDepthGrid;

  double gridReso;
  nodeHandle_.param("gridReso", gridReso, 0.02);
  gridReso_ = (float)gridReso;

  double gridWidth;
  nodeHandle_.param("gridWidth", gridWidth, 5000.0);
  gridWidth_ = int(gridWidth);

  double gridHeight;
  nodeHandle_.param("gridWidth", gridHeight, 5000.0);
  gridHeight_ = int(gridHeight);

  double useCamPose;
  nodeHandle_.param("useCamPose", useCamPose, 1.0);
  useCamPose_ = int(useCamPose);

  double fx,fy,cx,cy,width,height;
  nodeHandle_.param("fx", fx, 718.856);
  nodeHandle_.param("fy", fy, 718.856);
  nodeHandle_.param("cx", cx, 607.1928);
  nodeHandle_.param("cy", cy, 185.2157);
  nodeHandle_.param("width", width, 1241.0);
  nodeHandle_.param("height", height, 376.0);
  camInfo_.fx    = fx;
  camInfo_.fy    = fy;
  camInfo_.cx    = cx;
  camInfo_.cy    = cy;
  camInfo_.width = width;
  camInfo_.height = height;

  double updateRate;
  nodeHandle_.param("rate", updateRate, 0.0);
  if (updateRate == 0.0)
  {
    isContinousPublishing_ = false;
  }
  else
  {
    isContinousPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }

  if (!allParametersimage2PointCloud)
  {
    ROS_WARN("Could not image2PointCloud all parameters. Typical command-line usage:\n"
        "rosrun vid2depth_slam image2PointCloud"
        " _topic:=/my_topic"
        " _frame:=sensor_frame"
        " (optional: _rate:=publishing_rate)");
    return false;
  }

  return true;
}

void image2PointCloud::initialize()
{
  if (isContinousPublishing_)
  {
    tflistener_ = new tf::TransformListener(nodeHandle_);
    pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
    gridMapPublisher_    = nodeHandle_.advertise<nav_msgs::OccupancyGrid>(gridMapTopic_, 1, true);
    camPosePublisher_    = nodeHandle_.advertise<geometry_msgs::PoseStamped>(camPoseTopic_, 1, true);

    int q = 3;  // TODO: should has param subscriber_queue_size;
    imageRgbSub_         = new image_sub_type(nodeHandle_, rgbTopic_, q);
    depthSub_            = new image_sub_type(nodeHandle_, depthTopic_, q);

    camSync_             = new message_filters::Synchronizer<CameraPolicy>(CameraPolicy(q),
                                                                            *imageRgbSub_, 
                                                                            *depthSub_ );
    camSync_->registerCallback(boost::bind(&image2PointCloud::imageCallback, this
                                          , _1
                                          , _2
                                          )
                              );
    ROS_INFO_STREAM_NAMED("image2PointCloud", "Listening to " << rgbTopic_ << ", " << depthTopic_ );

    pointCloudMessage_->header.frame_id = "";

    timer_ = nodeHandle_.createTimer(updateDuration_, &image2PointCloud::timerCallback, this);
    isContinousPublishing_ = false;
  }
  else
  {
    Duration(1.0).sleep(); // Need this to get things image2PointCloudy before publishing.
    if (!publish()) ROS_ERROR("Something went wrong when trying to image2PointCloud and publish the point cloud file.");
    ros::requestShutdown();
  }
}

void image2PointCloud::timerCallback(const ros::TimerEvent& timerEvent)
{
  if (!publish()) ROS_ERROR("Something went wrong when trying to image2PointCloud and publish the point cloud file.");
}

bool image2PointCloud::publish()
{
  ros::Time now = Time::now();
  pointCloudMessage_->header.stamp = now;
  gridMap_->header.stamp           = now;
  camPose_->header.stamp           = now;
  if (isContinousPublishing_)
  {
    pointCloudPublisher_.publish(pointCloudMessage_);
    gridMapPublisher_.publish(gridMap_);
    camPosePublisher_.publish(camPose_);
    isContinousPublishing_ = false;
    ROS_INFO_STREAM("Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}

void image2PointCloud::updateCampose(tf::Quaternion& rot, tf::Vector3& translate){
  camPose_->header.frame_id    = pointCloudFrameId_;
  camPose_->pose.position.x    = translate.x();
  camPose_->pose.position.y    = translate.y();
  camPose_->pose.position.z    = translate.z();
  tf::Quaternion rot2 = rot * tf::Quaternion( 0, 0, 0.7068252, 0.7073883 );
  camPose_->pose.orientation.x = rot2.x();
  camPose_->pose.orientation.y = rot2.y();
  camPose_->pose.orientation.z = rot2.z();
  camPose_->pose.orientation.w = rot2.w();
}

void image2PointCloud::imageCallback (const sensor_msgs::ImageConstPtr& visual_img_msg
                                      ,const sensor_msgs::ImageConstPtr& depth_img_msg)
{
  ROS_INFO_STREAM_NAMED("image2PointCloud","Received data from camera");
  cv_bridge::CvImagePtr cv_rgb_ptr;
  cv_bridge::CvImagePtr cv_depth_ptr;

  try
  {
    cv_rgb_ptr       = cv_bridge::toCvCopy(visual_img_msg, sensor_msgs::image_encodings::BGR8);
    cv_depth_ptr     = cv_bridge::toCvCopy(depth_img_msg, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to point cloud from depth,rgb and cam info
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
                                                        util::createXZYRGBPointCloud(cv_depth_ptr->image,
                                                                              cv_rgb_ptr->image,
                                                                              zScaling_,
                                                                              maxDepth_,
                                                                              camInfo_)
                                                        );

  if(useCamPose_ > 0){
    // Get the camera pose of the current frame
    ros::Time timestamp = visual_img_msg->header.stamp;
    tflistener_->waitForTransform("world", "cam", timestamp, ros::Duration(0.005));
    tf::StampedTransform stampedTransform;
    tflistener_->lookupTransform("world", "cam", timestamp, stampedTransform);

    tf::Quaternion quat = stampedTransform.getRotation();
    tf::Quaternion rotXZY(quat.x(), quat.z(), -quat.y(), quat.w());
    tf::Vector3 translate(stampedTransform.getOrigin().x()/10, stampedTransform.getOrigin().z()/10, stampedTransform.getOrigin().y()/10);
    tf::Transform transform(rotXZY, translate);

    // Tranform point cloud to the target one base on received camera pose
    pcl_ros::transformPointCloud(*pointCloud, *pointCloudCur_, transform);

    // update camera pose published message
    updateCampose(rotXZY, translate);

    // Convert point cloud to grid map
    util::updateGrid(pointCloudCur_, cloudNormals_, gridMap_, transform, (double)maxDepthGrid_+stampedTransform.getOrigin().z()/10);
  }

  #ifdef ICP_POSE
    if(!initFlag_){
      pcl::copyPointCloud(*pointCloud,*pointCloudAcc_);
      initFlag_ = true;
    }else{
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
      sor.setStddevMulThresh (1.0);

      //filtering
      sor.setInputCloud (pointCloudCur_);
      sor.setMeanK(pointCloudCur_->size()/2);
      sor.filter (*pointCloudCur_);

      // The Iterative Closest Point algorithm
      int iterations = 20;  // Default number of ICP iterations
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      pcl::console::TicToc time;
      time.tic();
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setInputSource (pointCloudCur_);
      icp.setInputTarget (pointCloudAcc_);
      icp.setMaximumIterations (iterations);
      icp.setTransformationEpsilon (1e-9);
      icp.setMaxCorrespondenceDistance (0.05);
      icp.setEuclideanFitnessEpsilon (1);
      icp.setRANSACOutlierRejectionThreshold (1.5);
      icp.align (*pointCloudAcc_);
      std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : pointCloudCur_ -> pointCloudAcc_" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        util::print4x4Matrix (transformation_matrix);
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return;
      }
    }
    // Convert pcl point cloud to ros message
    toROSMsg(*pointCloudAcc_, *pointCloudMessage_);
  #else //!ICP_POSE
    if(useCamPose_ > 0)
      toROSMsg(*pointCloudCur_, *pointCloudMessage_);
    else  //!CAMERA_POSE
      toROSMsg(*pointCloud, *pointCloudMessage_);
  #endif //!ICP_POSE

  ROS_INFO_STREAM("Point cloud conversion done");

  pointCloudMessage_->header.frame_id = pointCloudFrameId_;

  isContinousPublishing_ = true;
}
}//namespace
