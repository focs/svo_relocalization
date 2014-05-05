
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
  int pyr_lvl = options_.pyr_lvl_;

  std::vector<std::vector<cv::KeyPoint>> query_keypoints (pyr_lvl+1);
  std::vector<std::vector<cv::KeyPoint>> best_match_keypoints (pyr_lvl+1);
  //if (frame_query->features_.size() <= 0)
  if (true)
  {
    // Calculate feature points for the query image
    std::cout << "Finding points in image1" << std::endl;
    FeatureDetector::FASTFindFeaturesPyr(frame_query->img_pyr_, pyr_lvl, query_keypoints);
    FeatureDetector::keyPointVectorToFrame(frame_query, query_keypoints);
  }
  else
  {
    // if points have already been calculated use them
    FeatureDetector::frameToKeyPointVector(query_keypoints, frame_query);
  }

  //if (frame_best_match->features_.size() <= 0)
  if (true)
  {
    // Calculate features for the best match image
    std::cout << "Finding points in image2" << std::endl;
    FeatureDetector::FASTFindFeaturesPyr(frame_best_match->img_pyr_, pyr_lvl, best_match_keypoints);
    FeatureDetector::keyPointVectorToFrame(frame_best_match, best_match_keypoints);
  }
  else
  {
    // if points have already been calculated use them
    FeatureDetector::frameToKeyPointVector(best_match_keypoints, frame_best_match);
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
  //
  std::cout << "pyr level" << pyr_lvl << std::endl;

  cv::Mat query_descriptors;
  cv::Mat best_match_descriptors;
  for (size_t i = 0; i <= pyr_lvl; ++i)
  {
    cv::Mat tmp;
    FeatureDetector::SURFExtractDescriptor(frame_query->img_pyr_.at(i), query_keypoints.at(i), tmp);
    std::cout << "merging: " << tmp.size() << std::endl;
    query_descriptors.push_back(tmp);
    std::cout << query_descriptors.size() << std::endl;

    FeatureDetector::SURFExtractDescriptor(frame_best_match->img_pyr_.at(i), best_match_keypoints.at(i), tmp);
    best_match_descriptors.push_back(tmp);
  }

  std::cout << query_descriptors.rows << "  " << query_descriptors.cols << std::endl;
  std::cout << best_match_descriptors.rows << "  " << query_descriptors.cols << std::endl;


  cv::FlannBasedMatcher matcher;
  //std::vector< cv::DMatch > matches;
  //matcher.match( query_descriptors, best_match_descriptors, matches );

  std::vector< std::vector< cv::DMatch>> knn_matches;
  matcher.knnMatch(query_descriptors, best_match_descriptors, knn_matches, 2);
  for (size_t i = 0; i < knn_matches.at(0).size(); ++i)
  {
    std::cout << "dist: " << knn_matches.at(0).at(i).distance << std::endl;
  }

  //std::cout << "Num of matches " << matches.size() << std::endl;


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
    //std::cout << "ratio: " <<  m.distance / knn_matches.at(i).at(1).distance << std::endl;
    if (m.distance / knn_matches.at(i).at(1).distance < 0.6)
    {
      if (m.distance <= 4*min_dist)
      {
        matches_used.push_back(m);
        int query_img_idx = m.queryIdx;
        int best_match_idx = m.trainIdx;

        //std::cout << "query_img_idx " << query_img_idx << " where sizez is " << frame_query->features_.size() << std::endl;
        //std::cout << "best_match_idx " << best_match_idx << " where sizez is " << frame_best_match->features_.size() << std::endl;
        opengv::bearingVector_t b;
        Eigen::Vector2d px = frame_query->features_.at(query_img_idx).px_;
        int pyr_lvl_found = frame_query->features_.at(query_img_idx).pyr_lvl_;
        //std::cout << "px before: " << px.transpose() << " with pyr_lvl " << pyr_lvl_found << std::endl;
        px << (static_cast<int>(px[0]) << pyr_lvl_found), (static_cast<int>(px[1]) << pyr_lvl_found);
        //std::cout << "px after: " << px.transpose() << std::endl;
        b << camera_model_->cam2world(px);
        im1_bearings.push_back(b);
        

        px = frame_best_match->features_.at(best_match_idx).px_;
        pyr_lvl_found = frame_best_match->features_.at(best_match_idx).pyr_lvl_;
        //std::cout << "bin2 px before: " << px.transpose() << " with pyr_lvl " << pyr_lvl_found << std::endl;
        px << (static_cast<int>(px[0]) << pyr_lvl_found), (static_cast<int>(px[1]) << pyr_lvl_found);
        //std::cout << "bin2 px after: " << px.transpose() << std::endl;
        b << camera_model_->cam2world(px);
        im2_bearings.push_back(b);

        //std::cout << "im1 " << (*--im1_bearings.end()).transpose() << std::endl;
        //std::cout << "im2 " << (*--im2_bearings.end()).transpose() << std::endl << std::endl;
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
  ransac.max_iterations_ = 5000;

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
  std::cout << "result matrix " << std::endl << se3_T_query_template.matrix() << std::endl;
  std::cout << "real matrix:  " << std::endl << (frame_query->T_frame_world_ * frame_best_match->T_frame_world_.inverse()).matrix() << std::endl;

  Eigen::Vector3d real_trans = (frame_query->T_frame_world_ * frame_best_match->T_frame_world_.inverse()).translation();
  Eigen::Vector3d found_trans = se3_T_query_template.translation();

  std::cout << "Real Translation:  " << (real_trans / real_trans.norm()).transpose() << std::endl;
  std::cout << "Found Translation: " << (found_trans / found_trans.norm()).transpose() << std::endl;
  std::cout << "error: " << (real_trans / real_trans.norm() - found_trans / found_trans.norm()).norm() << std::endl << std::endl;

  std::vector< cv::DMatch > matches_inilers;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    matches_inilers.push_back(matches_used.at(i));


  std::vector<cv::KeyPoint> query_keypoints_merged;
  for (size_t i = 0; i < query_keypoints.size(); ++i)
  {
    for (size_t j = 0; j < query_keypoints.at(i).size(); ++j)
    {
      cv::KeyPoint tmp_kp= query_keypoints.at(i).at(j);
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
      tmp_kp.pt = cv::Point2f(static_cast<int>(tmp_kp.pt.x) << i, static_cast<int>(tmp_kp.pt.y) << i);

      best_match_keypoints_merged.push_back(tmp_kp);
    }
  }

  cv::namedWindow("inliers", 1);
  cv::Mat img_matches;
  cv::drawMatches(frame_query->img_pyr_.at(0), query_keypoints_merged, frame_best_match->img_pyr_.at(0), best_match_keypoints_merged, matches_inilers, img_matches);
  cv::imshow("inliers", img_matches);

  cv::namedWindow("matches", 1);
  cv::drawMatches(frame_query->img_pyr_.at(0), query_keypoints_merged, frame_best_match->img_pyr_.at(0), best_match_keypoints_merged, matches_used, img_matches);
  cv::imshow("matches", img_matches);
  cv::waitKey(0);
  return se3_T_query_template * frame_best_match->T_frame_world_;

}


} /* reloc */ 


