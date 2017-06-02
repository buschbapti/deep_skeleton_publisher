#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "SkeletonTracking.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skeleton_tracking");
  ros::NodeHandle nh;
  
  SkeletonTracking skel(true);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect/registered", 1, &SkeletonTracking::imageCallback, &skel);
  ros::spin();
}