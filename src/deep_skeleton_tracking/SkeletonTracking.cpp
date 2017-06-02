#include "SkeletonTracking.hpp"

SkeletonTracking::SkeletonTracking(bool debug)
{
	this->debug = debug;
	if (debug) {
		cv::namedWindow("view");
  		cv::startWindowThread();
	}
}

SkeletonTracking::~SkeletonTracking()
{
	if (this->debug) {
		cv::destroyWindow("view");
	}
}

void SkeletonTracking::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		if (this->debug) {
			cv::imshow("view", cv_bridge::toCvShare(msg, "bgra8")->image);
			cv::waitKey(30);	
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgra8'.", msg->encoding.c_str());
	}
}