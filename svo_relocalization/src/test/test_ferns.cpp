
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <svo_relocalization/fern_classifier.h>

using namespace reloc;
using namespace std;
using namespace cv;

void warpImage (const cv::Mat &patch, cv::Mat &patch_warp)
{

  patch_warp = cv::Mat::zeros(patch.rows, patch.cols, patch.type());
  
  /// Compute a rotation matrix with respect to the center of the image
  cv::Point center = cv::Point( patch.cols/2, patch.rows/2 );
  double angle = (rand() % 3600) / 10.0;
  //angle = 0;
  double scale = 1;
  scale = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*0.5 + 0.75;

  cv::Mat warp_mat( 2, 3, CV_32FC1 );
  /// Get the rotation matrix with the specifications above
  warp_mat = getRotationMatrix2D( center, angle, scale );

  /// Rotate the warped image
  warpAffine( patch, patch_warp, warp_mat, patch_warp.size() );

  //cv::imshow("warp", patch_warp);
  //std::cout << "size: " << patch_warp.size() << std::endl;
  //cv::waitKey(0);

}

int main(int argc, char *argv[])
{
  srand((unsigned int)time(0));

  Mat image = imread("/opt/matlab2012a/toolbox/images/imdemos/peppers.png", CV_LOAD_IMAGE_GRAYSCALE);

  int max_x = 16;
  int max_y = 16;

  for (size_t num_tests_class = 8; num_tests_class < 14; ++num_tests_class)
  {
    cout << "Starting test with "  << num_tests_class << endl;
  for (size_t num_ferns_class = 5; num_ferns_class < 155; num_ferns_class+=5)
  {
  int right_test = 0;
  int test_count = 0;
  for (size_t t = 0; t < 100; ++t)
  {
    
  FernClassifier classifier (num_ferns_class, num_tests_class, max_x, max_y);

  Eigen::Matrix4Xi big_mat;
  big_mat.resize(4,100);

  for (size_t i = 0; i < big_mat.cols(); ++i)
  {
    big_mat(0,i) = rand() % image.cols;
    big_mat(1,i) = rand() % image.rows;
    big_mat(2,i) = 0;
    big_mat(3,i) = i;
  }

  //cout << big_mat << endl;

  vector<Mat> img_vector;
  img_vector.push_back(image);

  classifier.train(big_mat, img_vector);
  

  int classify_idx = 10;


  for (classify_idx = 0; classify_idx < big_mat.cols(); classify_idx++)
  {
    int x = big_mat(0, classify_idx);
    int y = big_mat(1, classify_idx);

    if (x - max_x > 0 &&
        x + max_x < img_vector.at(0).cols &&
        y - max_y > 0 &&
        y + max_y < img_vector.at(0).rows)
    {
      cv::Rect roi (x - max_x, y - max_y , max_x*2, max_y*2);

      //std::cout << "Rec roi: " << roi << std::endl << std::flush;
      //std::cout << image.rows << " " << image.cols << std::endl << std::flush;

      cv::Mat patch (image(roi));

      for (size_t i = 0; i < 100; ++i)
      {
        cv::Mat patch_warp;
        warpImage(patch, patch_warp);

        int real_class = big_mat(3, classify_idx);
        int found_class = classifier.classify(patch_warp);

        //cout << "Real class: " <<  real_class << endl;
        //cout << "Found class: " <<  found_class << endl;

        test_count++;
        if (real_class == found_class)
        {
          right_test++;
        }
      }
    }
  }

  }

  cout << "(" << num_ferns_class << "," << static_cast<float>(right_test)/static_cast<float>(test_count) << ")" << endl;
  
  }
  }

  return 0;
}
