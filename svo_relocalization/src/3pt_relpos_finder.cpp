

#include <svo_relocalization/3pt_relpos_finder.h>

#include <vector>
#include <Eigen/Core>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include <svo_relocalization/feature_detector.h>

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

  int pyr_lvl = options_.pyr_lvl_;

  std::vector<std::vector<cv::KeyPoint>> query_keypoints (pyr_lvl+1);
  std::vector<std::vector<cv::KeyPoint>> best_match_keypoints (pyr_lvl+1);

  FeatureDetector::getOpenCvFeatures(frame_query, pyr_lvl, query_keypoints);
  FeatureDetector::getOpenCvFeatures(frame_best_match, pyr_lvl, best_match_keypoints);

  std::cout << "pyr level" << pyr_lvl << std::endl;

  cv::Mat query_descriptors;
  cv::Mat best_match_descriptors;
  for (size_t i = 0; i <= pyr_lvl; ++i)
  {
    cv::Mat tmp;
    FeatureDetector::SURFExtractDescriptor(frame_query->img_pyr_.at(i), query_keypoints.at(i), tmp);
    query_descriptors.push_back(tmp);

    FeatureDetector::SURFExtractDescriptor(frame_best_match->img_pyr_.at(i), best_match_keypoints.at(i), tmp);
    best_match_descriptors.push_back(tmp);
  }

  std::cout << query_descriptors.rows << "  " << query_descriptors.cols << std::endl;
  std::cout << best_match_descriptors.rows << "  " << query_descriptors.cols << std::endl;


  cv::FlannBasedMatcher matcher;
  std::vector< std::vector< cv::DMatch>> knn_matches;
  matcher.knnMatch(query_descriptors, best_match_descriptors, knn_matches, 2);

  // Find min and max distance of the matches
  double max_dist = -std::numeric_limits<double>::min();
  double min_dist = std::numeric_limits<double>::max();
  for(size_t i = 0; i < knn_matches.size(); ++i)
  { 
    double dist = knn_matches.at(i).at(0).distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  std::cout << "Found max " << max_dist << std::endl;
  std::cout << "Found min " << min_dist << std::endl;



  opengv::bearingVectors_t im_bearings;
  opengv::points_t points;

  std::vector< cv::DMatch > matches_used;
  for (size_t i = 0; i < knn_matches.size(); ++i)
  {
    cv::DMatch m = knn_matches.at(i).at(0);
    if (m.distance / knn_matches.at(i).at(1).distance < 0.8)
    {
      if (m.distance <= 4*min_dist)
      {
        matches_used.push_back(m);
        int query_img_idx = m.queryIdx;
        int best_match_idx = m.trainIdx;

        opengv::bearingVector_t b;
        Eigen::Vector2d px = frame_query->features_.at(query_img_idx).px_;
        b << camera_model_->cam2world(px).normalized();
        im_bearings.push_back(b);
        

        opengv::point_t p;
        double depth = frame_best_match->features_.at(best_match_idx).depth_;
        px = frame_best_match->features_.at(best_match_idx).px_;
        p << frame_best_match->T_frame_world_.inverse() * 
          (camera_model_->cam2world(px).normalized() * depth);

        points.push_back(p);
      }
    }
  }


  // OpenGV part
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
  ransac.max_iterations_ = 500;

  ransac.computeModel();

  Sophus::SE3 T_world_query (ransac.model_coefficients_.leftCols(3), ransac.model_coefficients_.rightCols(1));

  // T_quey_best = T_query_world * T_world_best = T_query_world * T_best_world.inv()
  return T_world_query.inverse() * frame_best_match->T_frame_world_.inverse();

}


} /* reloc */ 

