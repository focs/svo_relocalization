
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se2.h>

#include <svo_relocalization/img_aling_se2.h>

using namespace cv;

int main(int argc, char const *argv[])
{

  printf("It does DO something!\n");

  Sophus::SE2 my_translation (0, Eigen::Vector2d(1,0));

  cv::Mat test_image = cv::imread(
      "/opt/matlab2012a/toolbox/images/imdemos/cameraman.tif", 
      CV_LOAD_IMAGE_GRAYSCALE);
  test_image.convertTo(test_image, CV_32F);

  // Warp template image
  cv::Mat warp_mat(2, 3, CV_32F);
  printf("warp_mat cols: %d rows: %d\n", warp_mat.cols, warp_mat.rows);
  cv::eigen2cv(
      Eigen::Matrix<double,2,3>(my_translation.matrix().topRows(2)),
      warp_mat);

  cv::Mat im_warp = cv::Mat::zeros(
      test_image.rows,
      test_image.cols,
      test_image.type());
  cv::warpAffine(test_image, im_warp, warp_mat, im_warp.size());

  //namedWindow( "Display window", WINDOW_AUTOSIZE );
  //imshow( "Display window", im_warp );

  //waitKey(0);

  SecondOrderMinimizationSE2 my_minimizer (test_image, im_warp);

  printf("nu_init: %f\n", my_minimizer.nu_init_);
  //printf("Image type: 

  Sophus::SE2 initial_model (0, Eigen::Vector2d(0,0));
  my_minimizer.optimize(initial_model);
  return 0;
}
