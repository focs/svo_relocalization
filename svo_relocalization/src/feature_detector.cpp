

#include <svo_relocalization/feature_detector.h>


namespace reloc
{
void FeatureDetector::FASTFindFeatures(
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &found_features)
{


  //Should not be called
  std::cout << "Error, calling fast" << std::endl;
  return;
  exit(-1);

  int threshold = 70; // ???? what is an adequate threshold 
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

void FeatureDetector::getOpenCvFeatures (
    const FrameSharedPtr frame,
    int pyr_lvl,
    std::vector<std::vector<cv::KeyPoint>> &found_features)
{

  if (frame->features_.size() <= 0)
  {
    std::cerr << "Looking for featured points" << std::endl;
    // Calculate feature points for the image
    FASTFindFeaturesPyr(frame->img_pyr_, pyr_lvl, found_features);
    keyPointVectorToFrame(frame, found_features);
  }
  else
  {
    // if points have already been calculated use them
    FeatureDetector::frameToKeyPointVector(found_features, frame);
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
      f.px_ << 
        (static_cast<int>(keypoints.at(i).at(j).pt.x) << i),
        (static_cast<int>(keypoints.at(i).at(j).pt.y) << i);
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
    Eigen::Vector2d px = frame->features_.at(i).px_;
    int pyr_lvl_found = frame->features_.at(i).pyr_lvl_;
    cv::KeyPoint p;
    p.pt = cv::Point2f(
          (static_cast<int>(px[0]) >> pyr_lvl_found),
          (static_cast<int>(px[1]) >> pyr_lvl_found));
    p.size = 7.0f;
    p.response = 90;
    //keypoints.at(frame->features_.at(i).pyr_lvl_).push_back(p);
    keypoints.at(pyr_lvl_found).push_back(p);
  }

}

void FeatureDetector::SURFExtractDescriptor (
    const cv::Mat &img,
    std::vector<cv::KeyPoint> &keypoints,
    cv::Mat &descriptors)
{

  cv::SiftDescriptorExtractor surfExtractor;
  //cv::SurfDescriptorExtractor surfExtractor;

  // Extract descriptors
  surfExtractor.compute( img, keypoints, descriptors);


}


} /*  reloc */ 
