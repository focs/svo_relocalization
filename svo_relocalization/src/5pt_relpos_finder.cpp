
#include <svo_relocalization/5pt_relpos_finder.h>

#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <vector>
#include <Eigen/Core>

#include <svo_relocalization/feature_detector.h>

namespace reloc
{
  
FivePtRelposFinder::FivePtRelposFinder (vk::AbstractCamera *camera_model):
  camera_model_(camera_model)
{

}

FivePtRelposFinder::~FivePtRelposFinder ()
{

}


void FivePtRelposFinder::removeFrame(int frame_id)
{

}

void FivePtRelposFinder::addFrame(const FrameSharedPtr &frame)
{

}

Sophus::SE3 FivePtRelposFinder::findRelpos(
    FrameSharedPtr frame_query,
    const FrameSharedPtr& frame_best_match)
{
  int pyr_lvl = 0;

  std::vector<cv::KeyPoint> query_keypoints;
  std::vector<cv::KeyPoint> best_match_keypoints;
  if (frame_query->features_.size() <= 0)
  {
    // Calculate feature points for the query image
    std::cout << "Finding points in image1" << std::endl;
    FeatureDetector::FASTFindFeatures(frame_query->img_pyr_.at(pyr_lvl), query_keypoints);
    FeatureDetector::keyPointVector_to_frame(frame_query, query_keypoints);
  }
  else
  {
    // if points have already been calculated use them
    FeatureDetector::frame_to_keyPointVector(query_keypoints, frame_query);
  }

  if (frame_best_match->features_.size() <= 0)
  {
    // Calculate features for the best match image
    std::cout << "Finding points in image2" << std::endl;
    FeatureDetector::FASTFindFeatures(frame_best_match->img_pyr_.at(pyr_lvl), best_match_keypoints);
    FeatureDetector::keyPointVector_to_frame(frame_best_match, best_match_keypoints);
  }
  else
  {
    // if points have already been calculated use them
    FeatureDetector::frame_to_keyPointVector(best_match_keypoints, frame_best_match);
  }

  //std::cout << "best_match_keypoints element " << std::endl
  // << best_match_keypoints.at(5).angle << std::endl 
  // << best_match_keypoints.at(5).size << std::endl 
  // << best_match_keypoints.at(5).class_id << std::endl 
  // << best_match_keypoints.at(5).response << std::endl 
  // << best_match_keypoints.at(5).octave << std::endl;

  //std::cout << "query_keypoints element " <<  std::endl
  // << query_keypoints.at(5).angle << std::endl 
  // << query_keypoints.at(5).size << std::endl 
  // << query_keypoints.at(5).class_id << std::endl 
  // << query_keypoints.at(5).response << std::endl 
  // << query_keypoints.at(5).octave << std::endl;

  cv::Mat query_descriptors;
  cv::Mat best_match_descriptors;
  FeatureDetector::SURFExtract_descriptor(frame_query->img_pyr_.at(pyr_lvl), query_keypoints, query_descriptors);
  FeatureDetector::SURFExtract_descriptor(frame_best_match->img_pyr_.at(pyr_lvl), best_match_keypoints, best_match_descriptors);

  std::cout << query_descriptors.rows << "  " << query_descriptors.cols << std::endl;
  std::cout << best_match_descriptors.rows << "  " << query_descriptors.cols << std::endl;


  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match( query_descriptors, best_match_descriptors, matches );

  std::cout << "Num of matches " << matches.size() << std::endl;

  // Find min and max distance of the matches
  double max_dist = -std::numeric_limits<double>::min();
  double min_dist = std::numeric_limits<double>::max();
  for(size_t i = 0; i < matches.size(); ++i)
  { 
    double dist = matches.at(i).distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }


  std::cout << "Found max " << max_dist << std::endl;
  std::cout << "Found min " << min_dist << std::endl;
  

  // create bearing vectors with found matches
  opengv::bearingVectors_t im1_bearings;
  opengv::bearingVectors_t im2_bearings;
  for (size_t i = 0; i < matches.size(); ++i)
  {
    if (matches.at(i).distance <= 30*min_dist)
    {
      int query_img_idx = matches.at(i).queryIdx;
      int best_match_idx = matches.at(i).trainIdx;

      //std::cout << "query_img_idx " << query_img_idx << " where sizez is " << frame_query->features_.size() << std::endl;
      //std::cout << "best_match_idx " << best_match_idx << " where sizez is " << frame_best_match->features_.size() << std::endl;

      opengv::bearingVector_t b;
      b << camera_model_->cam2world(frame_query->features_.at(query_img_idx).px_).normalized();
      im1_bearings.push_back(b);

      b << camera_model_->cam2world(frame_best_match->features_.at(best_match_idx).px_).normalized();
      im2_bearings.push_back(b);

      //std::cout << "im1 " << (*--im1_bearings.end()).transpose() << std::endl;
      //std::cout << "im2 " << (*--im2_bearings.end()).transpose() << std::endl << std::endl;
    }
  }

  std::cout << "num beanings " << im1_bearings.size() << std::endl;
  std::cout << "num beanings " << im2_bearings.size() << std::endl;


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

  Sophus::SE3 se3_T_query_template (ransac.model_coefficients_.leftCols(3), ransac.model_coefficients_.rightCols(1));

  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << "the number of inliers is: " << ransac.inliers_.size();
  std::cout << std::endl << std::endl;
  std::cout << "the found inliers are: " << std::endl;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    std::cout << ransac.inliers_[i] << " ";
  std::cout << std::endl << std::endl;

  std::cout << "result " << std::endl << se3_T_query_template << std::endl;

  return se3_T_query_template * frame_best_match->T_frame_world_;

}


} /* reloc */ 


