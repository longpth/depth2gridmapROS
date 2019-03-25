#include <ros/ros.h>
//TODO: Debug
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "image2PointCloud.h"

#ifdef TEST
static tf::TransformListener* tflistener_=NULL;

void imageCallbackMono(const sensor_msgs::ImageConstPtr& msg){
  ROS_INFO_STREAM_NAMED("Mono16","Receive Image mono 16");
  try
  {
    ros::Time depth_time = msg->header.stamp;
    tflistener_->waitForTransform("world", "cam", depth_time, ros::Duration(0.005));
    tf::StampedTransform transform;
    tflistener_->lookupTransform("world", "cam", depth_time, transform);
    printf("translate: %f %f %f\n",depth_time, transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    tf::Quaternion quat = transform.getRotation();
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    printf("rotation: %f %f %f\n",roll, pitch, yaw);
    cv::Mat depth = cv_bridge::toCvShare(msg, "mono16")->image;
    cv::imshow("depth", depth);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
  }
}

void imageCallbackBgr(const sensor_msgs::ImageConstPtr& msg){
  ROS_INFO_STREAM_NAMED("bgr8","Receive Image 8");
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
#endif

int main(int argc, char** argv){
  ros::init(argc, argv, "read");
  ros::NodeHandle nodeHandle("~");

  depth2gridmap::image2PointCloud image2PointCloud(nodeHandle);

  #ifdef TEST//TODO: Debug
  tflistener_ = new tf::TransformListener(nodeHandle);
  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber subMono = it.subscribe("/camera/depth/image_depth", 1, imageCallbackMono);
  image_transport::Subscriber subRgb = it.subscribe("/camera/rgb/image_color", 1, imageCallbackBgr);
  #endif

  ros::spin();

  #ifdef TEST
  delete tflistener_;
  #endif

  return 0;
}
