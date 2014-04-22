
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

  // Copy found points to the frame
  static void keyPointVector_to_frame(
      FrameSharedPtr frame,
      const std::vector<cv::KeyPoint> &keypoints);

  static void frame_to_keyPointVector(
      std::vector<cv::KeyPoint> &keypoints,
      const FrameSharedPtr frame);

  static void SURFExtract_descriptor (
      const cv::Mat &img,
      std::vector<cv::KeyPoint> &keypoints,
      cv::Mat &descriptors);

private:
  /* data */
};

} /* reloc */ 
#endif /* end of include guard: SVO_RELOCALIZER_FEATURE_DETECTOR_H_6G4QI5YT */

