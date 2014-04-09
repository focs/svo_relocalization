
#include <svo_relocalization/5pt_relpos_finder.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <vector>
#include <Eigen/Core>

namespace reloc
{
  
FivePtRelposFinder::FivePtRelposFinder ()
{

}

FivePtRelposFinder::~FivePtRelposFinder ()
{

}

Sophus::SE3 FivePtRelposFinder::findRelpos(
    const FrameSharedPtr& frame_query,
    const FrameSharedPtr& frame_best_match)
{

  std::vector<Eigen::Vector3d> im1_point_list;
  std::vector<Eigen::Vector3d> im2_point_list;

  opengv::bearingVectors_t im1_bearings;
  opengv::bearingVectors_t im2_bearings;
  std::copy(im1_point_list.begin(), im1_point_list.end(), std::back_inserter(im1_bearings));
  std::copy(im2_point_list.begin(), im2_point_list.end(), std::back_inserter(im2_bearings));


  opengv::relative_pose::CentralRelativeAdapter adapter (im1_bearings, im2_bearings);

  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem
      > ransac;
  
  boost::shared_ptr<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem> relposeproblem_ptr(
      new opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem(
      adapter,
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER));

  ransac.sac_model_ = relposeproblem_ptr;
  // Might need to be updated
  ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/800.0)));
  ransac.max_iterations_ = 50;

  ransac.computeModel();

  Sophus::SE3 result (ransac.model_coefficients_.rightCols(3), ransac.model_coefficients_.leftCols(1));

  return result;

}


} /* reloc */ 


