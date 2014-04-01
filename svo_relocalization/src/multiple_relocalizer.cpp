
#include <svo_relocalization/multiple_relocalizer.h>

#include <cstdio>

#include <svo_relocalization/CC_place_finder.h>
#include <svo_relocalization/ESM_relpos_finder.h>

namespace reloc
{
  
MultipleRelocalizer::MultipleRelocalizer (
    vk::AbstractCamera *camera_model,
    PlaceFinder place_finder_method,
    RelposFinder relpos_finder_method)
{
  switch (place_finder_method)
  {
    case CrossCorrelation:
      CCPlaceFinder *ccpf = new CCPlaceFinder();
      place_finder_ = ccpf;
      break;
  }

  switch (relpos_finder_method)
  {
    case ESM:
      ESMRelposFinder *esmrf = new ESMRelposFinder(camera_model);
      relpos_finder_ = esmrf;
      break;
  }

}

MultipleRelocalizer::~MultipleRelocalizer ()
{
  delete place_finder_;
  delete relpos_finder_;
}

void MultipleRelocalizer::addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id)
{

  //place_finder_->addFrame(img_pyr, T_frame_wordl, id);

}

bool MultipleRelocalizer::relocalize(
    const std::vector<cv::Mat>& query_img_pyr,
    const Sophus::SE3& T_frame_world_estimate,
    Sophus::SE3& T_frame_wordl_out,
   int& id_out)
{

}

} /* reloc */ 
