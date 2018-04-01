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

#include <memory>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "System.h"

#include "image_grabber.h"
#include "ros_mono.h"
#include "ros_rgbd.h"

const int kCameraMono = 0;
const int kCameraRgbd = 1;
const int kCameraStereo = 2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb2_slam_cuda_node");
    ros::start();

    ros::NodeHandle nh, nhPrivate("~");

    bool enableViewer = false;
    ImageGrabberConfig config;

    std::string vocabularyPath, settingsPath;
    if (!nhPrivate.getParam("vocabulary_path", vocabularyPath))
    {
      ROS_FATAL("vocabulary_path must be provided");
      return -1;
    }
    if (!nhPrivate.getParam("settings_path", settingsPath))
    {
      ROS_FATAL("settings_path must be provided");
      return -1;
    }
    nhPrivate.param<bool>("enable_viewer", enableViewer, false);
    nhPrivate.param<bool>("publish_tf", config.publishTf, false);
    nhPrivate.param<bool>("publish_pcl", config.publishPcl, false);
    nhPrivate.param<bool>("publish_pose", config.publishPose, true);
    nhPrivate.param<bool>("publish_image", config.publishImage, false);

    int nodeType = kCameraMono;
    nhPrivate.param<int>("camera_type", nodeType, kCameraMono);

    const bool enablePublishROSTopic = config.publishTf ||
      config.publishPcl || config.publishPose || config.publishImage;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::shared_ptr<ORB_SLAM2::System> SLAM;
    std::shared_ptr<ImageGrabber> igb;
    switch (nodeType)
    {
    case kCameraRgbd:
      SLAM = std::shared_ptr<ORB_SLAM2::System>(
          new ORB_SLAM2::System(vocabularyPath, settingsPath,
            ORB_SLAM2::System::RGBD, enableViewer));
      break;
    case kCameraMono:
      SLAM = std::shared_ptr<ORB_SLAM2::System>(
          new ORB_SLAM2::System(vocabularyPath, settingsPath,
            ORB_SLAM2::System::MONOCULAR, enableViewer));
      break;
    default:
      ROS_FATAL("invalid camera_type selected");
      return -1;
    }

    ORB_SLAM2::SlamData SLAMDATA(SLAM.get(), &nh, enablePublishROSTopic);

    switch (nodeType)
    {
    case kCameraRgbd:
      igb = std::shared_ptr<ImageGrabber>(
        new RgbdImageGrabber(SLAM.get(), &SLAMDATA, config, nhPrivate));
      break;
    case kCameraMono:
      igb = std::shared_ptr<ImageGrabber>(
        new MonoImageGrabber(SLAM.get(), &SLAMDATA, config, nhPrivate));
      break;
    default:
      ROS_FATAL("invalid camera_type selected");
      return -1;
    }

    ros::spin();

    // Stop all threads
    SLAM->Shutdown();

    ros::shutdown();

    return 0;
}

