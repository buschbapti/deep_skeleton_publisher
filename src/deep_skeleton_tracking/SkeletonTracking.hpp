#ifndef SKELETON__TRACKING_HPP
#define SKELETON__TRACKING_HPP

// C++ std library dependencies
#include <atomic> // std::atomic
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <cstdio> // sscanf
#include <string> // std::string
#include <thread> // std::this_thread
#include <vector> // std::vector
// OpenCV dependencies
#include <opencv2/core/core.hpp> // cv::Mat & cv::Size

// OpenPose dependencies
// Option a) Importing all modules
#include <openpose/headers.hpp>

// ROS dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Other 3rdpary depencencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging, CHECK, CHECK_EQ, LOG, VLOG, ...

class SkeletonTracking {
private:
	bool debug;
	op::PoseModel gflagToPoseModel(const std::string& poseModeString);
	std::tuple<cv::Size, cv::Size, cv::Size, op::PoseModel> gflagsToOpParameters();
public:
	SkeletonTracking(bool debug);
	~SkeletonTracking();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif // SKELETON__TRACKING_HPP