
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/img_aling_se2.h>
#include <svo_relocalization/se2_align_se3.h>

namespace reloc
{

ESMRelposFinder::ESMRelposFinder(vk::AbstractCamera *camera_model) :
  camera_model_(camera_model)
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
    FrameSharedPtr frame_query,
    const FrameSharedPtr& frame_best_match)
{

  // Optimizing from first parameter towards second
  SecondOrderMinimizationSE2 motion_estimator(
      frame_best_match->img_pyr_.at(options_.pyr_lvl_),
      frame_query->img_pyr_.at(options_.pyr_lvl_));
  
  // Model set to 0
  Sophus::SE2 se2_T_query_template;
  motion_estimator.optimize(se2_T_query_template);
  std::cout << "Found SE2: " << se2_T_query_template << std::endl;

  se2_T_query_template.translation() = se2_T_query_template.translation()*pow(2, options_.pyr_lvl_);

  // Apply found rotation to the known frame rotation
  Sophus::SE3 se3_T_query_template;
  se3_T_query_template = findSE3(
      se2_T_query_template,
      camera_model_,
      options_.n_iter_se2_to_se3_);

  std::cout << "Found SE3: " << se3_T_query_template << std::endl;

  Sophus::SE3 se3_T_query_template_final = frame_best_match->T_frame_world_;
  se3_T_query_template_final.so3() = se3_T_query_template.so3() * se3_T_query_template_final.so3();

  return se3_T_query_template_final;
  //return se3_T_query_template * frame_best_match->T_frame_world_;
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
