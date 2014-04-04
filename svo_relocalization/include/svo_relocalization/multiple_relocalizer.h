
#ifndef SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL
#define SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL

#include <svo_relocalization/abstract_relocalizer.h>

#include <svo_relocalization/abstract_place_finder.h>
#include <svo_relocalization/abstract_relpos_finder.h>

namespace vk
{
  class AbstractCamera;
} /* vk */ 

namespace reloc
{

struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector2d px; 
  double depth;
  size_t point_id;
};

struct FrameData {
  std::vector<Feature> features;
  std::vector<cv::Mat> img_pyr;
  Sophus::SE3 T_frame_world;
  size_t id;
};
typedef std::shared_ptr<FrameData> FrameDataPtr;

/// Implementation of a relocalizer using the technique from Klein and Murray
class MultipleRelocalizer : AbstractRelocalizer
{
  enum PlaceFinder { CrossCorrelation };
  enum RelposFinder { ESM };

public:
  MultipleRelocalizer(
      vk::AbstractCamera *camera_model,
      AbstractPlaceFinderPtr place_finder_method = CrossCorrelation,
      AbstractRelposFinderPtr relpos_finder_method = ESM);

  virtual ~MultipleRelocalizer ();

  void removeFrame(int frame_id)
  {
    palace_finder_->removeFrame(frame_id);
    relpos_finder_->removeFrame(frame_id);
  }
  
  void addFrame(FrameDataPtr frame)
  {
    palace_finder_->addFrame(frame);
    relpos_finder_->addFrame(frame);
  }

  bool relocalize(
      const std::vector<cv::Mat>& query_img_pyr,
      const Sophus::SE3& T_frame_world_estimate,
      Sophus::SE3& T_frame_wordl_out,
     int& id_out);

private:
  AbstractPlaceFinder *place_finder_;
  AbstractRelposFinder *relpos_finder_;
  
};

}

#endif /* end of include guard: SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL */
