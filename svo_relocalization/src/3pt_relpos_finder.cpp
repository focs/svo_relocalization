

#include <svo_relocalization/3pt_relpos_finder.h>

#include <vector>
#include <Eigen/Core>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

namespace reloc
{
  
ThreePtRelposFinder::ThreePtRelposFinder (vk::AbstractCamera *camera_model) :
  camera_model_(camera_model)
{

}

ThreePtRelposFinder::~ThreePtRelposFinder ()
{

}

void ThreePtRelposFinder::removeFrame(int frame_id)
{

}

void ThreePtRelposFinder::addFrame(const FrameSharedPtr &frame)
{

}

Sophus::SE3 ThreePtRelposFinder::findRelpos(
    FrameSharedPtr frame_query,
    const FrameSharedPtr& frame_best_match)
{

  std::vector<Eigen::Vector3d> im_point_list;
  std::vector<Eigen::Vector3d> world_points;

  opengv::bearingVectors_t im_bearings;
  std::copy(im_point_list.begin(), im_point_list.end(), std::back_inserter(im_bearings));

  opengv::points_t points;
  std::copy(world_points.begin(), world_points.end(), std::back_inserter(points));


  opengv::absolute_pose::CentralAbsoluteAdapter adapter (im_bearings, points);

  //Create an AbsolutePoseSac problem and Ransac
  //The method can be set to KNEIP, GAO or EPNP
  opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  boost::shared_ptr<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
      new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
      adapter,
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 50;

  ransac.computeModel();

  Sophus::SE3 result (ransac.model_coefficients_.rightCols(3), ransac.model_coefficients_.leftCols(1));

  return result;

}


} /* reloc */ 

