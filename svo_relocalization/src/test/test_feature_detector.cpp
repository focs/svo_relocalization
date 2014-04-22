

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.h>
#include <sophus/se2.h>
#include <vikit/atan_camera.h>
#include <svo_relocalization/5pt_relpos_finder.h>

using namespace reloc;
using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
  // Camera intrinsic parameters 
  float cam_width = 752;
  float cam_height = 480;
  Vector2d cam_size (cam_width, cam_height);
  float cam_fx = 0.582533;
  float cam_fy = 0.910057;
  float cam_cx = 0.510927;
  float cam_cy = 0.526193;
  float cam_d0 = 0.916379;

  vk::ATANCamera my_camera (cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, cam_d0); 

  Mat img1, img2;
  img1 = imread("/home/fox/catkin_ws/src/svo_relocalization/dense_input_data/image4882.png", CV_LOAD_IMAGE_GRAYSCALE);
  //img2 = imread("/home/fox/catkin_ws/src/svo_relocalization/dense_input_data/image4877.png", CV_LOAD_IMAGE_GRAYSCALE);

  //img1 = imread("/opt/matlab2012a/toolbox/images/imdemos/onion.png");


  Sophus::SE2 my_translation (0.01, Eigen::Vector2d(5,0));
 

  // Warp template image
  cv::Mat warp_mat(2, 3, CV_32F);
  cv::eigen2cv(
      Eigen::Matrix<double,2,3>(my_translation.matrix().topRows(2)),
      warp_mat);

  cv::warpAffine(img1, img2, warp_mat, img1.size(), INTER_LINEAR, BORDER_CONSTANT, 0);

  FrameSharedPtr f1 (new Frame());
  FrameSharedPtr f2 (new Frame());
  f1->img_pyr_.push_back(img1);
  f2->img_pyr_.push_back(img2);

  FivePtRelposFinder pos_finder(&my_camera);

  Sophus::SE3 found_T_template_query;
  found_T_template_query = pos_finder.findRelpos(f1, f2);

  

  return 0;
}
