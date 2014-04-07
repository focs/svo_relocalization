
#ifndef SVO_RELOCALIZER_FRAME_H_WJTE2OVF
#define SVO_RELOCALIZER_FRAME_H_WJTE2OVF

#include <memory>
#include <Eigen/Core>
#include <vector>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

namespace reloc
{
  
struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d px_; 
  double depth_;
  size_t point_id_;

  Feature () : 
    px_(Eigen::Vector2d(0.0, 0.0)),
    depth_(0.0),
    point_id_(0)
  {}

  Feature (const Eigen::Vector2d& px, double depth, size_t point_id) :
    px_(px),
    depth_(depth),
    point_id_(point_id)
  {}


};

class Frame
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  std::vector<Feature> features_;
  std::vector<cv::Mat> img_pyr_;
  Sophus::SE3 T_frame_world_;
  size_t id_;

};

typedef std::shared_ptr<Frame> FrameSharedPtr;

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_FRAME_H_WJTE2OVF */
