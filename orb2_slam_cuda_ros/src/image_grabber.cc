#include "image_grabber.h"

ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM,
    ORB_SLAM2::SlamData* pSLAMData, const ImageGrabberConfig& config): 
  mpSLAM(pSLAM), mpSLAMDATA(pSLAMData), publishTf(config.publishTf),
  publishPcl(config.publishPcl), publishPose(config.publishPose)
{}

void ImageGrabber::MaybePublishROS(cv_bridge::CvImageConstPtr cv_ptr, cv::Mat& Tcw)
{
  if (mpSLAMDATA->EnablePublishROSTopics())
  {
    if (publishTf)
    {
      mpSLAMDATA->PublishTFForROS(Tcw, cv_ptr);
    }
    if (publishPose)
    {
      mpSLAMDATA->PublishPoseForROS(cv_ptr);
    }
    if (publishPcl)
    {
      mpSLAMDATA->PublishPointCloudForROS();
    }
    if (publishImage)
    {
      mpSLAMDATA->PublishCurrentFrameForROS();
    }
  }
}

