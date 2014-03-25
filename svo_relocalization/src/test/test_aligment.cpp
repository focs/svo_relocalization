
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se2.h>

#include <svo_relocalization/img_aling_se2.h>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{

  printf("It does DO something!\n");

  Sophus::SE2 my_translation (0.051, Eigen::Vector2d(5,5));
 
  cv::Mat test_image = cv::imread(
      "/opt/matlab2012a/toolbox/images/imdemos/cameraman.tif", 
      CV_LOAD_IMAGE_GRAYSCALE);
  cv::resize(test_image, test_image, Size(50,50));
  test_image.convertTo(test_image, CV_32F);

  Sophus::SE2 center_translation (0, Eigen::Vector2d(-test_image.cols/2, -test_image.rows/2));
  Sophus::SE2 tmp_transf = center_translation.inverse() * my_translation * center_translation;


  // Warp template image
  cv::Mat warp_mat(2, 3, CV_32F);
  printf("warp_mat cols: %d rows: %d\n", warp_mat.cols, warp_mat.rows);
  cv::eigen2cv(
      Eigen::Matrix<double,2,3>(tmp_transf.matrix().topRows(2)),
      warp_mat);

  cout << "tmp_trans " << tmp_transf << endl;
  cout << "Used modle " << warp_mat << endl;

  cv::Mat im_warp = cv::Mat::ones(
      test_image.rows,
      test_image.cols,
      test_image.type());
  cv::warpAffine(test_image, im_warp, warp_mat, im_warp.size(), INTER_LINEAR, BORDER_CONSTANT, 0);
  //cout << im_warp << endl;

  namedWindow( "Display window", WINDOW_AUTOSIZE );
  imshow( "Display window", im_warp );
  waitKey(0);

  reloc::SecondOrderMinimizationSE2 my_minimizer (im_warp, test_image);

  printf("nu_init: %f\n", my_minimizer.nu_init_);
  //printf("Image type: 

  Sophus::SE2 initial_model (0, Eigen::Vector2d(0,0));
  my_minimizer.optimize(initial_model);
  std::cout << "final model: " << initial_model << std::endl;
  return 0;
}
