

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

void FeatureDetector::FASTFindFeaturesPyr(
    const std::vector<cv::Mat> &imgs,
    int pyr_lvl,
    std::vector<std::vector<cv::KeyPoint>> &found_features)
{
  for (size_t i = 0; i <= pyr_lvl; ++i)
  {
    FASTFindFeatures(imgs.at(i), found_features.at(i));
  }
}

void FeatureDetector::keyPointVectorToFrame(
    FrameSharedPtr frame,
    const std::vector<std::vector<cv::KeyPoint>> &keypoints)
{
  
  for (size_t i = 0; i < keypoints.size(); ++i)
  {
    for (size_t j = 0; j < keypoints.at(i).size(); ++j)
    {
      Feature f;
      f.px_ << keypoints.at(i).at(j).pt.x, keypoints.at(i).at(j).pt.y;
      f.pyr_lvl_ = i;
      frame->features_.push_back(f);
    }
  }

}

void FeatureDetector::frameToKeyPointVector(
    std::vector<std::vector<cv::KeyPoint>> &keypoints,
    const FrameSharedPtr frame)
{
  
  for (size_t i = 0; i < frame->features_.size(); ++i)
  {
    cv::KeyPoint p;
    p.pt = cv::Point2f(frame->features_.at(i).px_[0], frame->features_.at(i).px_[1]);
    p.size = 7.0f;
    p.response = 90;
    keypoints.at(frame->features_.at(i).pyr_lvl_).push_back(p);
  }

}

void FeatureDetector::SURFExtractDescriptor (
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors)
{

  cv::SurfDescriptorExtractor surfExtractor;

  // Extract descriptors
  surfExtractor.compute( img, keypoints, descriptors);


}


} /*  reloc */ 
