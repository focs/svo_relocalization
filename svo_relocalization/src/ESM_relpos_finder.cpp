
#include <svo_relocalization/ESM_relpos_finder.h>
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
  
/// Returns the SE3 transformation between the two images
Sophus::SE3 ESMRelposFinder::findRelpos (cv::Mat query_img, cv::Mat template_img)
{

  SecondOrderMinimizationSE2 motion_estimator(query_img, template_img);
  
  // Model set to 0
  Sophus::SE2 se2_T_template_query;
  //std::cout << "Template transformation: " << T_template_query << std::endl;
  motion_estimator.optimize(se2_T_template_query);

  // Apply found rotation to the known frame rotation
  Sophus::SE3 se3_T_template_query;
  se3_T_template_query = findSE3(se2_T_template_query, camera_model_).inverse();

  return se3_T_template_query;
}

Sophus::SE3 ESMRelposFinder::findSE3(Sophus::SE2 t, vk::AbstractCamera *camera_model)
{
  Sophus::SE3 model;
  SE2toSE3 s23 (t, camera_model);
  s23.n_iter_ = 3;
  s23.optimizeGaussNewton(model);

  return model;
}

} /* reloc */ 
