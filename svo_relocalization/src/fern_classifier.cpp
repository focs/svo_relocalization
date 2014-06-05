
#include <svo_relocalization/fern_classifier.h>

namespace reloc
{

FernClassifier::FernClassifier (
  int num_ferns,
  int num_tests,
  int max_x,
  int max_y)
{

  Nr_ = 1;
  num_warps_ = 100;

  for (size_t i = 0; i < num_ferns; ++i)
  {
    ferns.push_back(Fern(num_tests, max_x, max_y));
  }

  distributions.resize(num_ferns);
}

FernClassifier::~FernClassifier ()
{

}


void FernClassifier::warpImage (const cv::Mat &patch, cv::Mat &patch_warp)
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

void FernClassifier::train (
    const Eigen::Matrix4Xi &data,
    const std::vector<cv::Mat> &images)
{

  // K might be big....
 uint32_t K = 1 << (ferns.at(0).getNumTests());

  // Hopefully not too many classes
  int num_classes = data.row(3).maxCoeff()+ 1;
  std::cout << "Number of classes: " << num_classes << std::endl;

  int max_x = ferns.at(0).getMaxX()*2;
  int max_y = ferns.at(0).getMaxY()*2;
  
  class_count.resize(num_classes);
  class_count.fill(0);

  // Initialize matrices to num_classes x num_possible_fern_eval (2^num_tests)
  for (size_t i = 0; i < distributions.size(); ++i)
  {
    distributions.at(i).resize(num_classes, K+1);
    distributions.at(i).fill(0);
  }



  for (size_t i = 0; i < data.cols(); ++i)
  {
    int x = data(0, i);
    int y = data(1, i);
    int img_idx = data(2, i);
    int class_idx = data(3, i);

    if (x - max_x/2 > 0 &&
        x + max_x/2 < images.at(img_idx).cols &&
        y - max_y/2 > 0 &&
        y + max_y/2 < images.at(img_idx).rows)
    {
      cv::Rect roi (x - max_x/2, y - max_y/2, max_x, max_y);
      cv::Mat patch (images.at(img_idx)(roi));

      for (size_t j = 0; j < num_warps_; ++j)
      {

        cv::Mat patch_warp;
        warpImage(patch, patch_warp);

        //cv::imshow("image", images.at(img_idx));
        //cv::imshow("patch", patch);
        //cv::waitKey(0);

        // One more view of the class
        class_count(class_idx)++;

        // Evaluate all ferns with patch
        for (size_t fern_idx = 0; fern_idx < ferns.size(); ++fern_idx)
        {
          uint32_t evaluated_fern = ferns.at(fern_idx).evaluetePatch(patch_warp);
          //std::cout << "evaluated_fern: " << evaluated_fern << std::endl;
          
          // fern j evaluates tp evalueated_fern with patch
          //std::cout << "size distribution: " <<
          //  distributions.at(fern_idx).rows() << " x " <<
          //  distributions.at(fern_idx).cols() << std::endl;

          //std::cout << "class_idx " << class_idx << std::endl;
          distributions.at(fern_idx)(class_idx, evaluated_fern)++;
        }
      }
    }
  }
  
  /*
  for (size_t i = 0; i < distributions.size(); ++i)
  {
    std::cout << "Distributions " << i << std::endl << distributions.at(i) << std::endl;
  }
  */
  //std::cout << "Class_count" << std::endl << class_count.transpose() << std::endl;

  //std::cout << distributions.at(4).row(0) << std::endl;
  //std::cout << class_count(0) << std::endl;
  //exit(-1);
 

}

int FernClassifier::classify (const cv::Mat &patch)
{
  uint32_t K = 1 << ferns.at(0).getNumTests();

  // Initialize final distribution
  Eigen::VectorXd final_dsitribution;
  final_dsitribution.resize(class_count.size());
  final_dsitribution.fill(1);

  for (size_t i = 0; i < ferns.size(); ++i)
  {
    uint32_t evaluated_fern = ferns.at(i).evaluetePatch(patch);

    //            N_{k,c} + Nr
    // p_{k,c} = --------------
    //            N_c + K x Nr
    
    final_dsitribution =
      final_dsitribution.array() *
      //(distributions.at(i).col(evaluated_fern).cast<float>().array() + Nr_)
      // / (class_count.cast<float>().array() + K*Nr_);
      (distributions.at(i).col(evaluated_fern).cast<double>().array() + Nr_);
       /// (class_count.cast<double>().array() + K*Nr_ );
    //std::cout << final_dsitribution.transpose() << std::endl;


    //std::cout << final_dsitribution.sum() << std::endl;

    
  }

  //std::cout << "sum " << (distributions.at(20).row(0).cast<double>()/class_count(0)).sum() << std::endl;


  final_dsitribution =
    final_dsitribution.array() /
    (class_count.cast<float>().array() +K*Nr_ ).cast<double>();

  //std::cout << "Found dist: " << std::endl << final_dsitribution.transpose() << std::endl;

  //std::cout << "max of dist " << final_dsitribution.maxCoeff() << std::endl;
  int class_idx;
  final_dsitribution.maxCoeff(&class_idx);

  return class_idx;

}

} /* reloc */ 
