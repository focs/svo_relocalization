
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
  Eigen::Vector3d point_w_;
  size_t point_id_;
  size_t pyr_lvl_;

  Feature () : 
    px_(Eigen::Vector2d(0,0)),
    point_w_(0,0,0),
    point_id_(-1)
  {}

  Feature (const Eigen::Vector2d& px, const Eigen::Vector3d &point_w, size_t point_id) :
    px_(px),
    point_w_(point_w),
    point_id_(point_id)
  {}


};

class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<Feature> features_;
  std::vector<cv::Mat> img_pyr_;
  Sophus::SE3 T_frame_world_;
  size_t id_;

  static void createPyr (
      const cv::Mat &im,
      int num_lvl,
      std::vector<cv::Mat> &pyr_out)
  {
    pyr_out.push_back(im);

    for (size_t i = 1; i < num_lvl; ++i)
    {
      cv::Size s (pyr_out[i-1].cols/2, pyr_out[i-1].rows/2);
      cv::Mat resized;
      cv::resize(pyr_out[i-1], resized, s);
      pyr_out.push_back(resized);
    }
  }
};

typedef std::shared_ptr<Frame> FrameSharedPtr;

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_FRAME_H_WJTE2OVF */
