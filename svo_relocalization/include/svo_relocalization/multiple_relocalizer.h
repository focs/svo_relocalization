
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

/// Implementation of a relocalizer using the technique from Klein and Murray
class MultipleRelocalizer : AbstractRelocalizer
{
  enum PlaceFinder { CrossCorrelation };
  enum RelposFinder { ESM };

public:
  MultipleRelocalizer (
      vk::AbstractCamera *camera_model,
      PlaceFinder place_finder_method = CrossCorrelation,
      RelposFinder relpos_finder_method = ESM);

  virtual ~MultipleRelocalizer ();

  void addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id);

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
