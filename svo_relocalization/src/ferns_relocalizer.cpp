

#include <svo_relocalization/ferns_relocalizer.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

namespace reloc
{

FernsRelocalizer::FernsRelocalizer(
    vk::AbstractCamera *cam,
    int num_ferns,
    int num_tests,
    int max_x,
    int max_y) :
  fern_classifier(num_ferns, num_tests, max_x, max_y),
  cam_(cam),
  frame_counter(0),
  point_counter(0),
  hasTrained(false)
{

}

FernsRelocalizer::~FernsRelocalizer ()
{

}

void FernsRelocalizer::removeFrame(int frame_id)
{

}
  
void FernsRelocalizer::addFrame(FrameSharedPtr frame)
{

  std::cout << "Adding frame id: " << frame->id_ << std::endl;
  
  frame_idx2id[frame_counter] = frame->id_;
  frame_id2idx[frame->id_] = frame_counter++;
  frames.push_back(frame);
  
  for (auto it_feature = frame->features_.cbegin();
      it_feature != frame->features_.cend();
      ++it_feature)
  {

    if ((*it_feature).point_id_ == -1)
      continue;

    int point_id = (*it_feature).point_id_;
    // Find if key is in map
    if (point_id2idx.find(point_id) == point_id2idx.end())
    {
      //point not in map yet
      point_id2idx[point_id] = point_counter;
      point_id2pose[point_id] = (*it_feature).point_w_;
      point_idx2id[point_counter++] = point_id;
    }
    else{
      //std::cout << "point already used :)" << std::endl;
    }

    Eigen::Vector4i col;
    col(0) = (*it_feature).px_(0);
    col(1) = (*it_feature).px_(1);
    col(2) = frame_id2idx[frame->id_];
    col(3) = point_id2idx[point_id];

    train_data.push_back(col);
  }
}

void FernsRelocalizer::train ()
{
  Eigen::Matrix4Xi data;
  std::vector<cv::Mat> images;

  data.resize(4,train_data.size());
  for (size_t i = 0; i < train_data.size(); ++i)
  {
    data.col(i) = train_data.at(i);
  }

  for (size_t i = 0; i < frames.size(); ++i)
  {
    images.push_back(frames.at(i)->img_pyr_.at(0));
  }

  fern_classifier.train(data, images);

  hasTrained = true;
}

bool FernsRelocalizer::relocalize(
      FrameSharedPtr frame_query,
      Sophus::SE3 &pose_out,
      int &id_out)
{

  if(!hasTrained)
  {
    std::cerr << "Error: can not relocalize before training" << std::endl;
    return false;
  }

  int max_x = fern_classifier.getMaxX() * 2;
  int max_y = fern_classifier.getMaxY() * 2;


  opengv::bearingVectors_t im_bearings;
  opengv::points_t points;
  for (auto it_feature = frame_query->features_.cbegin();
      it_feature != frame_query->features_.cend();
      ++it_feature)
  {
    int x = (*it_feature).px_(0);
    int y = (*it_feature).px_(1);

    if (x - max_x/2 > 0 &&
        x + max_x/2 < frame_query->img_pyr_.at(0).cols &&
        y - max_y/2 > 0 &&
        y + max_y/2 < frame_query->img_pyr_.at(0).rows)
    {

      cv::Rect roi ( x - max_x/2, y - max_y /2, max_x, max_y);
      cv::Mat patch (frame_query->img_pyr_.at(0)(roi));

      // Classify! do the magic.
      int found_class = fern_classifier.classify(patch);
      
      Eigen::Vector2d px (x, y);
      im_bearings.push_back(cam_->cam2world(px).normalized());

      points.push_back(point_id2pose[point_idx2id[found_class]]);
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

  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << "the number of inliers is: " << ransac.inliers_.size() << " out of " << points.size();
  std::cout << std::endl << std::endl;
  std::cout << "the found inliers are: " << std::endl;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    std::cout << ransac.inliers_[i] << " ";
  std::cout << std::endl << std::endl;

  Sophus::SE3 T_world_query (ransac.model_coefficients_.leftCols(3), ransac.model_coefficients_.rightCols(1));

  FrameSharedPtr closest_frame = findClosestFrame(T_world_query.inverse());

  id_out = closest_frame->id_;
  pose_out =
    T_world_query.inverse() *
    closest_frame->T_frame_world_.inverse();


  return ransac.inliers_.size() > 3;
}

FrameSharedPtr FernsRelocalizer::findClosestFrame(Sophus::SE3 pose)
{
  double min_dist = std::numeric_limits<double>::max();
  FrameSharedPtr closest_frame;

  for (auto it_frame = frames.cbegin();
      it_frame != frames.cend();
      it_frame++)
  {
    double dist =
      ((*it_frame)->T_frame_world_.inverse().translation() -
       pose.inverse().translation()).norm();

    if (dist < min_dist) 
    {
      min_dist = dist;
      closest_frame = *it_frame;
    }
  }

  return closest_frame;
}

} /* reloc */ 
