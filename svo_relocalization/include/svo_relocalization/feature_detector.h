
#ifndef SVO_RELOCALIZER_FEATURE_DETECTOR_H_6G4QI5YT
#define SVO_RELOCALIZER_FEATURE_DETECTOR_H_6G4QI5YT

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <svo_relocalization/frame.h>

namespace reloc
{
class FeatureDetector
{
public:

  // Find featured points in the image
  static void FASTFindFeatures (
      const cv::Mat &frame,
      std::vector<cv::KeyPoint> &found_features);

  static void FASTFindFeaturesPyr(
      const std::vector<cv::Mat> &imgs,
      int pyr_lvl,
      std::vector<std::vector<cv::KeyPoint>> &found_features);

  static void getOpenCvFeatures (
      const FrameSharedPtr frame,
      int pyr_lvl,
      std::vector<std::vector<cv::KeyPoint>> &found_features);

  // Copy found points to the frame
  static void keyPointVectorToFrame(
      FrameSharedPtr frame,
      const std::vector<std::vector<cv::KeyPoint>> &keypoints);

  static void frameToKeyPointVector(
      std::vector<std::vector<cv::KeyPoint>> &keypoints,
      const FrameSharedPtr frame);

  static void SURFExtractDescriptor (
      const cv::Mat &img,
      std::vector<cv::KeyPoint> &keypoints,
      cv::Mat &descriptors);

private:
  /* data */
};

} /* reloc */ 
#endif /* end of include guard: SVO_RELOCALIZER_FEATURE_DETECTOR_H_6G4QI5YT */

