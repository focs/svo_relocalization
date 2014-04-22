

#include <svo_relocalization/feature_detector.h>


namespace reloc
{
void FeatureDetector::FASTFindFeatures(
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &found_features)
{

  int threshold = 50; // ???? what is an adequate threshold 
  bool non_maximal_spresion = true;
  cv::FAST(img, found_features, threshold, non_maximal_spresion);

}

void FeatureDetector::keyPointVector_to_frame(
    FrameSharedPtr frame,
    const std::vector<cv::KeyPoint> &keypoints)
{
  
  for (size_t i = 0; i < keypoints.size(); ++i)
  {
    Feature f;
    f.px_ << keypoints.at(i).pt.x, keypoints.at(i).pt.y;
    frame->features_.push_back(f);
  }

}

void FeatureDetector::frame_to_keyPointVector(
    std::vector<cv::KeyPoint> &keypoints,
    const FrameSharedPtr frame)
{
  
  for (size_t i = 0; i < frame->features_.size(); ++i)
  {
    cv::KeyPoint p;
    p.pt = cv::Point2f(frame->features_.at(i).px_[0], frame->features_.at(i).px_[1]);
    p.size = 7.0f;
    p.response = 90;
    keypoints.push_back(p);
  }

}

void FeatureDetector::SURFExtract_descriptor (
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors)
{

  cv::SurfDescriptorExtractor surfExtractor;

  // Extract descriptors
  surfExtractor.compute( img, keypoints, descriptors);


}


} /*  reloc */ 
