#ifndef ROS_RGBD_H
#define ROS_RGBD_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "System.h"

#include "slam_data.h"
#include "image_grabber.h"

class RgbdImageGrabber: public ImageGrabber
{
public:
  RgbdImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMData,
      const ImageGrabberConfig& config, ros::NodeHandle &nh);
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
private:
  ros::Subscriber sub;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync;
};

#endif
