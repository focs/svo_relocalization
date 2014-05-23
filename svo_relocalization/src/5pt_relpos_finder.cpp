
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
  // Get mean depth
  float depth_sum, mean_depth;
  //for (size_t i = 0; i < frame_best_match->features_.size(); ++i)
  //{
  //  depth_sum += frame_best_match->features_.at(i).depth_;
  //}
  //mean_depth = depth_sum / frame_best_match->features_.size();

  int pyr_lvl = options_.pyr_lvl_;

  std::vector<std::vector<cv::KeyPoint>> query_keypoints (
      frame_query->img_pyr_.size());
  std::map<size_t,Feature*> query_feature_phash2ptr;
  std::vector<std::vector<cv::KeyPoint>> best_match_keypoints (
      frame_best_match->img_pyr_.size());
  std::map<size_t,Feature*> best_match_feature_phash2ptr;

  FeatureDetector::frameToKeyPointVector(
      query_keypoints,
      query_feature_phash2ptr,
      frame_query);
  FeatureDetector::frameToKeyPointVector(
      best_match_keypoints,
      best_match_feature_phash2ptr,
      frame_best_match);

  int point_count;
  // Create map idx to cvpoint
  std::map<int,cv::KeyPoint*>query_point_idx2ptr;
  point_count = 0;
  for (size_t i = 0; i < query_keypoints.size(); ++i)
  {
    for (size_t j = 0; j < query_keypoints.at(i).size(); ++j)
    {
      query_point_idx2ptr[point_count++] = &query_keypoints.at(i).at(j);
    }
  }

  // Create map idx to cvpoint
  std::map<int,cv::KeyPoint*>best_match_point_idx2ptr;
  point_count = 0;
  for (size_t i = 0; i < best_match_keypoints.size(); ++i)
  {
    for (size_t j = 0; j < best_match_keypoints.at(i).size(); ++j)
    {
      best_match_point_idx2ptr[point_count++] = &best_match_keypoints.at(i).at(j);
    }
  }

  cv::Mat query_descriptors;
  cv::Mat best_match_descriptors;
  for (size_t i = 0; i <= pyr_lvl; ++i)
  {
    cv::Mat tmp;
    FeatureDetector::SURFExtractDescriptor(
        frame_query->img_pyr_.at(i),
        query_keypoints.at(i),
        tmp);
    query_descriptors.push_back(tmp);

    FeatureDetector::SURFExtractDescriptor(
        frame_best_match->img_pyr_.at(i),
        best_match_keypoints.at(i),
        tmp);
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
  

  // create bearing vectors with found matches
  opengv::bearingVectors_t im1_bearings;
  opengv::bearingVectors_t im2_bearings;
  std::vector< cv::DMatch > matches_used;
  for (size_t i = 0; i < knn_matches.size(); ++i)
  {
    cv::DMatch m = knn_matches.at(i).at(0);
    if (m.distance / knn_matches.at(i).at(1).distance < 0.8)
    {
      if (m.distance <= 4*min_dist)
      {
        int query_idx = m.queryIdx;
        int best_match_idx = m.trainIdx;

        Feature *query_feature;
        Feature *best_match_feature;

        // find pointers to referenced feature (this is fucked up)
        query_feature = query_feature_phash2ptr[
          query_point_idx2ptr[query_idx]->hash()];
        best_match_feature = best_match_feature_phash2ptr[
          best_match_point_idx2ptr[best_match_idx]->hash()];

        matches_used.push_back(m);

        opengv::bearingVector_t b;
        Eigen::Vector2d px = query_feature->px_;
        b << camera_model_->cam2world(px).normalized();
        im1_bearings.push_back(b);
        

        px = best_match_feature->px_;
        b << camera_model_->cam2world(px).normalized();
        im2_bearings.push_back(b);
      }
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
  //ransac.threshold_ = 0.00000001;
  ransac.max_iterations_ = 500;

  ransac.computeModel();

  Sophus::SE3 se3_T_query_template (ransac.model_coefficients_.leftCols(3), ransac.model_coefficients_.rightCols(1));

  // Apply mean depth
  //se3_T_query_template.translation() = se3_T_query_template.translation() / se3_T_query_template.translation().norm() * mean_depth;

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
  std::cout << "result matrix " << std::endl << se3_T_query_template.matrix() << std::endl;
  std::cout << "real matrix:  " << std::endl << (frame_query->T_frame_world_ * frame_best_match->T_frame_world_.inverse()).matrix() << std::endl;

  Eigen::Vector3d real_trans = (frame_query->T_frame_world_ * frame_best_match->T_frame_world_.inverse()).translation();
  Eigen::Vector3d found_trans = se3_T_query_template.translation();

  std::cout << "Real Translation:  " << (real_trans / real_trans.norm()).transpose() << std::endl;
  std::cout << "Found Translation: " << (found_trans / found_trans.norm()).transpose() << std::endl;
  std::cout << "error: " << (real_trans / real_trans.norm() - found_trans / found_trans.norm()).norm() << std::endl << std::endl;

  /**********************************TEST**************************************/
  std::vector< cv::DMatch > matches_inilers;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    matches_inilers.push_back(matches_used.at(ransac.inliers_.at(i)));


  std::vector<cv::KeyPoint> query_keypoints_merged;
  for (size_t i = 0; i < query_keypoints.size(); ++i)
  {
    for (size_t j = 0; j < query_keypoints.at(i).size(); ++j)
    {
      cv::KeyPoint tmp_kp= query_keypoints.at(i).at(j);
      //tmp_kp.pt = cv::Point2f(tmp_kp.pt.x, tmp_kp.pt.y);
      tmp_kp.pt = cv::Point2f(static_cast<int>(tmp_kp.pt.x) << i, static_cast<int>(tmp_kp.pt.y) << i);

      query_keypoints_merged.push_back(tmp_kp);
    }
  }

  std::vector<cv::KeyPoint> best_match_keypoints_merged;
  for (size_t i = 0; i < best_match_keypoints.size(); ++i)
  {
    for (size_t j = 0; j < best_match_keypoints.at(i).size(); ++j)
    {
      cv::KeyPoint tmp_kp= best_match_keypoints.at(i).at(j);
      //tmp_kp.pt = cv::Point2f(tmp_kp.pt.x, tmp_kp.pt.y);
      tmp_kp.pt = cv::Point2f(static_cast<int>(tmp_kp.pt.x) << i, static_cast<int>(tmp_kp.pt.y) << i);

      best_match_keypoints_merged.push_back(tmp_kp);
    }
  }

  cv::namedWindow("inliers", 1);
  cv::Mat img_matches;
  cv::drawMatches(
      frame_query->img_pyr_.at(0),
      query_keypoints_merged,
      frame_best_match->img_pyr_.at(0),
      best_match_keypoints_merged,
      matches_inilers,
      img_matches);

  cv::imshow("inliers", img_matches);

  cv::namedWindow("matches", 1);
  cv::drawMatches(
      frame_query->img_pyr_.at(0),
      query_keypoints_merged,
      frame_best_match->img_pyr_.at(0),
      best_match_keypoints_merged,
      matches_used,
      img_matches);
  cv::imshow("matches", img_matches);
  cv::waitKey(0);

  /**********************************TEST**************************************/

  return se3_T_query_template.inverse();

}


} /* reloc */ 


