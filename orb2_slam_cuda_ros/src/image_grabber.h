#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"

#include "slam_data.h"

struct ImageGrabberConfig {
  bool publishTf;
  bool publishPcl;
  bool publishPose;
  bool publishImage;
};

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::SlamData* pSLAMData, const ImageGrabberConfig& config);
  void MaybePublishROS(cv_bridge::CvImageConstPtr cv_ptr, cv::Mat& Tcw);
  virtual ~ImageGrabber() {}
protected:
  ORB_SLAM2::System* mpSLAM;
  ORB_SLAM2::SlamData* mpSLAMDATA;

  bool publishTf;
  bool publishPcl;
  bool publishPose;
  bool publishImage;
};

#endif
