
#ifndef KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5
#define KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5

#include <list>

#include <svo_relocalization/virtual_relocalizer.h>

class KMRelocalizer : VirtualRelocalizer
{
  typedef std::pair<cv::Mat, Sophus::SE3> ImagePose;

public:
  KMRelocalizer ();
  virtual ~KMRelocalizer ();

  void addFrame(const cv::Mat& img, const Sophus::SE3& pose, int id);

  bool relocalize(
      const cv::Mat& img,
      const Sophus::SE3& last_pose,
      Sophus::SE3& pose_out,
      int& id_out);

private:
  /// Find best match with small blured images
  ImagePose& findBestMatch(const cv::Mat& queryImage);

  std::list<ImagePose> images_; //<! List of images included so far
};

#endif /* end of include guard: KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5 */

