
#ifndef VIRTUAL_RELOCALIZER_H_RA9MYHW8
#define VIRTUAL_RELOCALIZER_H_RA9MYHW8

#include <opencv2/opencv.hpp>
#include <sophus/se3.h> 

/// Abstract Class (interface) to implement a relocalization method
class VirtualRelocalizer
{
public:
  VirtualRelocalizer (){};
  virtual ~VirtualRelocalizer (){};

  /// Add new frame to the relocalizer (usually all keyframes are used here)
  virtual void addFrame (const cv::Mat& img, const Sophus::SE3& pose, int id) = 0;
  
  /// Start relocalization.
  virtual bool relocalize(
      const cv::Mat& img,
      const Sophus::SE3& last_pose,
      Sophus::SE3& pose_out,
      int& id_out) = 0;

};


#endif /* end of include guard: VIRTUAL_RELOCALIZATOR_H_RA9MYHW8 */

