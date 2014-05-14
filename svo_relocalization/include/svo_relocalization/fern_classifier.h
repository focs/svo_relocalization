
#ifndef SVO_RELOCALIZER_FERN_CLASSIFIER_H_EVJCHV98
#define SVO_RELOCALIZER_FERN_CLASSIFIER_H_EVJCHV98

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

#include <svo_relocalization/fern.h>

namespace reloc
{

class FernClassifier
{
public:
  FernClassifier (
    int num_ferns,
    int num_tests,
    int max_x,
    int max_y);

  virtual ~FernClassifier ();

  static void warpImage (const cv::Mat &patch, cv::Mat &patch_warp);

  int getMaxX () { return ferns.at(0).getMaxX(); };
  int getMaxY () { return ferns.at(0).getMaxY(); };

  // Matrix 4xNum_features
  // -------------------------------------
  // | x         | ... | ... | ......... |
  // | y         | ... | ... | ......... |
  // | img_idx   | ... | ... | ......... |
  // | point_idx | ... | ... | ......... |
  // -------------------------------------
  

  void train (
      const Eigen::Matrix4Xi &data,
      const std::vector<cv::Mat> &images);

  // Returns 3D point id (class)
  int classify (const cv::Mat &patch);

private:

  std::vector<Fern> ferns;

  // One matrix per fern
  // Every matrix is num_classes x 2^num_tests
  // A column of the matrix is the count of class (number of times the class
  // evaluates to this fern)
  std::vector<Eigen::MatrixXi> distributions;

  // Used to normalize
  Eigen::VectorXi class_count;

  // Regularization term
  float Nr_;
  int num_warps_;
};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_FERN_CLASSIFIER_H_EVJCHV98 */

