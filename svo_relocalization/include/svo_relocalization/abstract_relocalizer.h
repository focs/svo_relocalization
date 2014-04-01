
#ifndef SVO_RELOCALIZATION_ABSTRACT_RELOCALIZER_H_RA9MYHW8
#define SVO_RELOCALIZATION_ABSTRACT_RELOCALIZER_H_RA9MYHW8

#include <opencv2/opencv.hpp>
#include <sophus/se3.h> 

namespace reloc
{

/// Abstract Class (interface) to implement a relocalization method
class AbstractRelocalizer
{
public:
  AbstractRelocalizer (){};
  virtual ~AbstractRelocalizer (){};

  /// Add new frame to the relocalizer (usually all keyframes are used here)
  virtual void addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_world, int id) = 0;
  
  /// Start relocalization.
  virtual bool relocalize(
      const std::vector<cv::Mat>& query_img_pyr,
      const Sophus::SE3& T_frame_world_estimate,
      Sophus::SE3& T_frame_wordl_out,
      int& id_out) = 0;

};


} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZATION_ABSTRACT_RELOCALIZATOR_H_RA9MYHW8 */

