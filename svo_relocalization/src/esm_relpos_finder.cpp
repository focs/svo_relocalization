
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/img_aling_se2.h>
#include <svo_relocalization/se2_align_se3.h>

namespace reloc
{

ESMRelposFinder::ESMRelposFinder(vk::AbstractCamera *camera_model) :
  camera_model_(camera_model),
  pyr_lvl_(0)
{

}

ESMRelposFinder::~ESMRelposFinder()
{

}
  
void ESMRelposFinder::removeFrame(int frame_id)
{
}

void ESMRelposFinder::addFrame(const FrameSharedPtr &frame)
{
}


Sophus::SE3 ESMRelposFinder::findRelpos(
    const FrameSharedPtr& frame_query,
    const FrameSharedPtr& frame_best_match,
    const Sophus::SE3& T_frame_query_estimate)
{

  SecondOrderMinimizationSE2 motion_estimator(
      frame_query->img_pyr_.at(options_.pyr_lvl_),
      frame_best_match->img_pyr_.at(options_.pyr_lvl_));
  
  // Model set to 0
  Sophus::SE2 se2_T_template_query;
  motion_estimator.optimize(se2_T_template_query);

  // Apply found rotation to the known frame rotation
  Sophus::SE3 se3_T_template_query;
  se3_T_template_query = findSE3(
      se2_T_template_query,
      camera_model_,
      options_.n_iter_se2_to_se3_).inverse();

  return se3_T_template_query;
}

Sophus::SE3 ESMRelposFinder::findSE3(
    Sophus::SE2 t,
    vk::AbstractCamera *camera_model,
    uint32_t n_iter)
{
  Sophus::SE3 model;
  SE2toSE3 s23 (t, camera_model);
  s23.n_iter_ = n_iter;
  s23.optimizeGaussNewton(model);

  return model;
}

} /* reloc */ 
