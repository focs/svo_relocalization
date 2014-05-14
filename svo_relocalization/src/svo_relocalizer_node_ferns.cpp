// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/package.h>
#include <string>
#include <svo/frame.h>
#include <svo/frame_handler_mono.h>
#include <svo/feature.h> //Quim
#include <svo/point.h> //Quim
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

#include <svo/feature_detection.h>
#include <svo_relocalization/multiple_relocalizer.h>
#include <svo_relocalization/cc_place_finder.h>
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/5pt_relpos_finder.h>
#include <svo_relocalization/3pt_relpos_finder.h>
#include <svo_relocalization/empty_relpos_finder.h>
#include <svo_relocalization/feature_detector.h>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include <svo_relocalization/fern_classifier.h>

namespace svo {

using namespace std;
using namespace vk;
using namespace std;
using namespace cv;

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  vk::UserInputThread* user_input_thread_;
  ros::Subscriber sub_remote_key_;
  string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;

  //Relocalizer
  reloc::MultipleRelocalizer *relocalizer_;

  map<int,int> frame_idx2id;
  map<int,int> frame_id2idx;
  map<int,int> point_idx2id;
  map<int,int> point_id2idx;
  map<int,Point*> point_id2ptr;

  reloc::FernClassifier fern_classifier;

  bool hasTrained;

  void train();
  Sophus::SE3 relocalize(FramePtr frame);
  FramePtr findClosestFrame(Sophus::SE3 pose);
  //end Relocalizer


  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);


};

VoNode::
VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  user_input_thread_(new vk::UserInputThread()),
  remote_input_(""),
  cam_(NULL),
  quit_(false),
  fern_classifier(50, 14, 32, 32),
  hasTrained(false)
{

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");


  // Set initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));
  
  // Init camera
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::
~VoNode()
{
  delete vo_;
  delete user_input_thread_;
  delete cam_;
  delete relocalizer_;
}

void VoNode::
imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();

  cv::imshow("current frame", img);
  cv::waitKey(1);

  if (vo_->stage() == FrameHandlerMono::STAGE_RELOCALIZING)
  {
    
    if (!hasTrained)
    {
      printf("ERROR: trying to relocalize before training\n");
      exit(-1);
    }

    FramePtr frame(new Frame(cam_, img, msg->header.stamp.toSec()));
    feature_detection::FastDetector detector(
        img.cols,
        img.rows,
        Config::gridSize(),
        Config::nPyrLevels());

    detector.detect(
        frame.get(),
        frame->img_pyr_,
        Config::triangMinCornerScore(),
        frame->fts_);

      Sophus::SE3 found_T_frame_world = relocalize(frame);
      FramePtr closest_frame = findClosestFrame(found_T_frame_world);

      vo_->relocalizeFrameAtPose(
          closest_frame->id_,
          found_T_frame_world * closest_frame->T_f_w_.inverse(),
          img,
          msg->header.stamp.toSec());
  }
  else
  {
    vo_->addImage(img, msg->header.stamp.toSec());
    if (hasTrained)
    {
      //cout << "Real pose" << endl << vo_->lastFrame()->T_f_w_ << endl;
      //cout << "Found pose" << endl << relocalize(vo_->lastFrame()) << endl;
    }

  }

  //if (hasTrained != true && vo_->map().keyframes_.size() >= 10)
  //  train();

  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());



  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000); // avoid busy loop when paused
}

FramePtr VoNode::findClosestFrame(Sophus::SE3 pose)
{
  double min_dist = std::numeric_limits<double>::max();
  FramePtr closest_frame;

  for (auto it_frame = vo_->map().keyframes_.cbegin();
      it_frame != vo_->map().keyframes_.cend();
      it_frame++)
  {
    double dist = ((*it_frame)->T_f_w_.translation() - pose.translation()).norm();
    if (dist < min_dist) 
    {
      min_dist = dist;
      closest_frame = *it_frame;
    }
  }

  return closest_frame;
}

Sophus::SE3 VoNode::relocalize(FramePtr frame)
{

  
  int max_x = fern_classifier.getMaxX() * 2;
  int max_y = fern_classifier.getMaxY() * 2;


  opengv::bearingVectors_t im_bearings;
  opengv::points_t points;
  for (auto it_feature = frame->fts_.cbegin();
      it_feature != frame->fts_.cend();
      ++it_feature)
  {
    int x = (*it_feature)->px(0);
    int y = (*it_feature)->px(1);

    if (x - max_x/2 > 0 &&
        x + max_x/2 < frame->img_pyr_.at(0).cols &&
        y - max_y/2 > 0 &&
        y + max_y/2 < frame->img_pyr_.at(0).rows)
    {

      cv::Rect roi ( x - max_x/2, y - max_y /2, max_x, max_y);
      cv::Mat patch (frame->img_pyr_.at(0)(roi));

      // Classify! do the magic.
      int found_class = fern_classifier.classify(patch);
      
      Eigen::Vector2d px (x, y);
      im_bearings.push_back(cam_->cam2world(px).normalized());

      Point *p;
      p = point_id2ptr[point_idx2id[found_class]];

      points.push_back(p->pos_);
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

  return T_world_query.inverse();


}

void VoNode::train()
{
  vector<Eigen::Vector4i> tmp;
  Eigen::Matrix4Xi data;
  vector<Mat> images;

  // Test shit
  //FramePtr test_frame;
  //int frame_test_indx = 3;
  // end

  cout << "starting training" <<endl;

  int frame_counter = 0;
  int point_counter = 0;
  cout << "Number or frames: " << vo_->map().keyframes_.size() << endl;

  for (auto it_frame = vo_->map().keyframes_.cbegin();
      it_frame != vo_->map().keyframes_.cend();
      it_frame++)
  {
    cout << "Training with framel frame id: " << (*it_frame)->id_ << endl;
    
    frame_idx2id[frame_counter] = (*it_frame)->id_;
    frame_id2idx[(*it_frame)->id_] = frame_counter++;
    images.push_back((*it_frame)->img_pyr_.at(0));
    
    int cont = 0;
    for (auto it_feature = (*it_frame)->fts_.cbegin();
        it_feature != (*it_frame)->fts_.cend();
        ++it_feature)
    {

      if ((*it_feature)->point == NULL)
        continue;

      int point_id = (*it_feature)->point->id_;
      if (point_id2idx.find(point_id) == point_id2idx.end())
      {
        //point not in map yet
        point_id2idx[point_id] = point_counter;
        point_id2ptr[point_id] = (*it_feature)->point;
        point_idx2id[point_counter++] = point_id;
        //cout << "point already used :)" << endl;
      }

      Eigen::Vector4i col;
      col(0) = (*it_feature)->px(0);
      col(1) = (*it_feature)->px(1);
      col(2) = frame_id2idx[(*it_frame)->id_];
      col(3) = point_id2idx[point_id];

      //if (frame_id2idx[(*it_frame)->id_] != frame_test_indx)
      tmp.push_back(col);
      //else
      //  test_frame = *it_frame;
    }

  }

  data.resize(4,tmp.size());
  for (size_t i = 0; i < tmp.size(); ++i)
  {
    data.col(i) = tmp.at(i);
  }

  //cout << data << endl;
  //cout << "number of images: " << images.size() << endl;

  fern_classifier.train(data, images);

  hasTrained = true;

/*************************  TEST  *********************************************/
  //int max_x = fern_classifier.getMaxX() * 2;
  //int max_y = fern_classifier.getMaxY() * 2;

  //int count_good = 0;
  //int count_tests = 0;

  //cv::Mat image;

  ////FramePtr test_frame = (*(--vo_->map().keyframes_.end()));
  ////test_frame = (*it);
  //image = test_frame->img_pyr_.at(0);


  //cout << "Test frame id: " << test_frame->id_ << endl;

  //opengv::bearingVectors_t im_bearings;
  //opengv::points_t points;
  //for (auto it_feature = test_frame->fts_.cbegin();
  //      it_feature != test_frame->fts_.cend();
  //      ++it_feature)
  //{
  //  int x = (*it_feature)->px(0);
  //  int y = (*it_feature)->px(1);
  //  cv::Rect roi (x - max_x/2, y - max_y /2, max_x, max_y);

  //  if ((*it_feature)->point == NULL)
  //      continue;

  //  if (x - max_x/2 > 0 &&
  //          x + max_x/2 < image.cols &&
  //          y - max_y/2 > 0 &&
  //          y + max_y/2 < image.rows)
  //  {
  //    std::cout << "Rec roi: " << roi << std::endl << std::flush;

  //    cv::Mat patch (image(roi));

  //    int real_class = point_id2idx[(*it_feature)->point->id_];
  //    int found_class = fern_classifier.classify(patch);

  //    cout << "real class: " << real_class << " found class: " << found_class << endl;
  //    
  //    count_tests++;
  //    if (real_class == found_class)
  //    {
  //      count_good++;
  //    }
  //    
  //    Eigen::Vector2d px (x, y);
  //    im_bearings.push_back(cam_->cam2world(px).normalized());

  //    Point *p;
  //    p = point_id2ptr[point_idx2id[found_class]];

  //    points.push_back(p->pos_);

  //  }
  //}

  //cout << "ratio of good matches: " << (double)count_good/(double)count_tests << endl;


  //// OpenGV part
  //opengv::absolute_pose::CentralAbsoluteAdapter adapter (im_bearings, points);

  ////Create an AbsolutePoseSac problem and Ransac
  ////The method can be set to KNEIP, GAO or EPNP
  //opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  //boost::shared_ptr<
  //    opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
  //    new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
  //    adapter,
  //    opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
  //ransac.sac_model_ = absposeproblem_ptr;
  //ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  //ransac.max_iterations_ = 5000;

  //ransac.computeModel();

  //std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  //std::cout << "the number of inliers is: " << ransac.inliers_.size() << " out of " << points.size();
  //std::cout << std::endl << std::endl;
  //std::cout << "the found inliers are: " << std::endl;
  //for(size_t i = 0; i < ransac.inliers_.size(); i++)
  //  std::cout << ransac.inliers_[i] << " ";
  //std::cout << std::endl << std::endl;

  //Sophus::SE3 T_query_world (ransac.model_coefficients_.leftCols(3), ransac.model_coefficients_.rightCols(1));

  //cout << "Found T: " << endl << T_query_world.inverse() << endl;
  //cout << "Real T: " << endl << test_frame->T_f_w_ << endl;
  //cout << "Found T relocalize:" << endl << relocalize(image) << endl;


  //exit(-1);

/*****************  END     TEST  *********************************************/

  cout << "end training" << endl;
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  char console_input = user_input_thread_->getInput();
  if(console_input != 0)
    input = console_input;

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("Svo User Input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("Svo User Input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("Svo User Input: START\n");
      break;
    case 't':
      printf("Svo User Input: TRAIN\n");
      train();
      break;

    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  svo::VoNode vo_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("Svo terminated.\n");
  return 0;
}
