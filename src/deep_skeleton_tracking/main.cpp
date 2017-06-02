#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "SkeletonTracking.hpp"
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging, CHECK, CHECK_EQ, LOG, VLOG, ...

int main(int argc, char **argv)
{
	ros::init(argc, argv, "skeleton_tracking");
	ros::NodeHandle nh;

	// Initializing google logging (Caffe uses it for logging)
    google::InitGoogleLogging("skeleton_tracking");

   	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

  
	SkeletonTracking skel(true);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/kinect/registered", 1, &SkeletonTracking::imageCallback, &skel);
	ros::spin();
}