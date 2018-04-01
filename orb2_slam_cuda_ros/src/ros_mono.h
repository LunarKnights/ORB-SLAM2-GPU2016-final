#ifndef ROS_MONO_H
#define ROS_MONO_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include "System.h"

#include "slam_data.h"
#include "image_grabber.h"

class MonoImageGrabber: public ImageGrabber
{
public:
  MonoImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMData,
      const ImageGrabberConfig& config, ros::NodeHandle &nh);
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
private:
  ros::Subscriber sub;
};

#endif
