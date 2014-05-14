
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <svo_relocalization/fern_classifier.h>

using namespace reloc;
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
  srand((unsigned int)time(0));

  Mat image = imread("/home/fox/Documents/vibot/masterThesisZurich/relocalizer_1/data/train_reloc_1_2014-04-24_17-41-22/image7088.png");

  int max_x = 16;
  int max_y = 16;

  FernClassifier classifier (100, 11, max_x, max_y);

  Eigen::Matrix4Xi big_mat;
  big_mat.resize(4,30);

  for (size_t i = 0; i < big_mat.cols(); ++i)
  {
    big_mat(0,i) = rand() % image.cols;
    big_mat(1,i) = rand() % image.rows;
    big_mat(2,i) = 0;
    big_mat(3,i) = i;
  }

  cout << big_mat << endl;

  vector<Mat> img_vector;
  img_vector.push_back(image);

  classifier.train(big_mat, img_vector);
  

  int classify_idx = 0;
  int x = big_mat(0, classify_idx);
  int y = big_mat(1, classify_idx);


  cv::Rect roi (x - max_x/2, y - max_y /2, max_x, max_y);

  std::cout << "Rec roi: " << roi << std::endl << std::flush;
  std::cout << image.rows << " " << image.cols << std::endl << std::flush;

  cv::Mat patch (image(roi));
  
  cout << "Real class: " << big_mat(3, classify_idx) << endl;
  cout << "Found class: " << classifier.classify(patch) << endl;

  return 0;
}
