/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "System.h"

#include "ros_rgbd.h"

RgbdImageGrabber::RgbdImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMData,
      const ImageGrabberConfig& config, ros::NodeHandle &nh):
  ImageGrabber(pSLAM, pSLAMData, config),
  rgb_sub(nh, "rgb_raw", 1),
  depth_sub(nh, "depth_raw", 1),
  sync(sync_pol(10), rgb_sub, depth_sub)
{
    sync.registerCallback(boost::bind(&RgbdImageGrabber::GrabRGBD, this, _1, _2));
}

void RgbdImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
  // Saves 3 points of time to calculate fps: begin, finish cv process and finish SLAM process
  mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_BEGIN);
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_CV_PROCESS);

  cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

  mpSLAMDATA->SaveTimePoint(ORB_SLAM2::SlamData::TimePointIndex::TIME_FINISH_SLAM_PROCESS);

  mpSLAMDATA->CalculateAndPrintOutProcessingFrequency();

  if (Tcw.empty()) {
    return;
  }

  MaybePublishROS(cv_ptrRGB, Tcw);
}

