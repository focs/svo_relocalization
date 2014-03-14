
#ifndef IMG_ALIGN_SE2_CPP
#define IMG_ALIGN_SE2_CPP

#include <iostream>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <svo_relocalization/img_aling_se2.h>

namespace reloc
{
  
SecondOrderMinimizationSE2::SecondOrderMinimizationSE2 (const cv::Mat& im, const cv::Mat& im_template)
{
  //im.copyTo(im_);
  // Copy and convert to float
  im.convertTo(im_, CV_32F);
  //im_template.copyTo(im_template_);
  // Copy and convert to float
  im_template.convertTo(im_template_, CV_32F);

  cv::Sobel(im_template_, im_template_grad_x_, CV_32F, 1, 0, 1);
  cv::Sobel(im_template_, im_template_grad_y_, CV_32F, 0, 1, 1);

  method_ = SecondOrderMinimizationSE2::GaussNewton;

}

SecondOrderMinimizationSE2::~SecondOrderMinimizationSE2 ()
{

}

double SecondOrderMinimizationSE2::computeResiduals (
    const Sophus::SE2& model, 
    bool linearize_system, 
    bool compute_weight_scale)
{
  std::cout << "Current model" << std::endl << model << std::endl;

  Sophus::SE2 center_translation (0, Eigen::Vector2d(-im_.cols/2, -im_.rows/2));
  Sophus::SE2 temp_model = center_translation.inverse() * model * center_translation;
  // Warp template image
  cv::Mat warp_mat(2, 3, CV_32F);
  cv::eigen2cv(
      Eigen::Matrix<double,2,3>(temp_model.matrix().topRows(2)),
      warp_mat);

  cv::Mat im_warp = -cv::Mat::ones(im_.rows, im_.cols, im_.type());
  //cv::warpAffine(im_, im_warp, warp_mat, im_warp.size());
  cv::warpAffine(im_, im_warp, warp_mat, im_warp.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

  // Compute warped image gradient
  cv::Mat im_warp_grad_x;
  cv::Mat im_warp_grad_y;
  cv::Sobel(im_warp, im_warp_grad_x, CV_32F, 1, 0, 1);
  cv::Sobel(im_warp, im_warp_grad_y, CV_32F, 0, 1, 1);

  // Compute mean grad
  cv::Mat im_grad_mean_x = 0.25*(im_template_grad_x_ + im_warp_grad_y);
  cv::Mat im_grad_mean_y = 0.25*(im_template_grad_y_ + im_warp_grad_y);

  // Compute error image
  cv::Mat im_error = im_template_ - im_warp;

  float chi2 = 0;
  int num_pix = 0;
  Jres_.setZero();
  for (int v = 0; v < im_.rows; ++v)
  {
    for (int u = 0; u < im_.cols; ++u)
    {
      
      if (im_warp.at<float>(v,u) > 0)
      {
        if (linearize_system)
        {
          // Jacovian/Steepest image
          // Angle ~ 0 so sin(0) = 0 and cos(0) = 1
          // J(2) = (-xsin(a) + ycos(a))*grad_x + (-xcos(a) - ysin(a))*grad_y
          Eigen::Vector3d J;
          J(0) = im_grad_mean_x.at<float>(v,u);
          J(1) = im_grad_mean_y.at<float>(v,u);
          J(2) = -(v-im_.rows/2)*im_grad_mean_x.at<float>(v,u) + (u-im_.cols/2)*im_grad_mean_y.at<float>(v,u); 

          //printf("J angle: %f, x: %f, y: %f\n", J(0), J(1), J(2));
          // Compute lower triangle of H
          for (size_t i = 0; i < J.size(); ++i)
          {
            for (size_t j = 0; j <= i; ++j)
            {
              H_(i,j) += J(i) * J(j);
            }
          }

          // Compute Jres
          Jres_ += J*im_error.at<float>(v,u);
          //printf("im error %f\n", im_error.at<float>(v,u));
          //printf("Jres angle: %f, x: %f, y: %f\n", Jres_(0), Jres_(1), Jres_(2));
        }
      

        // Compute total error
        chi2 += im_error.at<float>(v,u) * im_error.at<float>(v,u);
        num_pix++;
      }
    }
  }

  if (linearize_system)
  {
    // Fill uper triangle of H
    for (size_t i = 1; i < H_.cols(); ++i)
    {
      for (size_t j = 0; j < i; ++j)
      {
        H_(j,i) = H_(i,j);
      }
    }
  }

  //chi2 = cv::sum(im_error.mul(im_error))[0];
  //return chi2/(im_.rows*im_.cols);
  return chi2/num_pix;
  
}


int SecondOrderMinimizationSE2::solve()
{
  x_ = H_.ldlt().solve(-Jres_);
  //printf("Jres angle: %f, x: %f, y: %f\n", Jres_(0), Jres_(1), Jres_(2));
  //printf("angle: %f, x: %f, y: %f\n", x_(0), x_(1), x_(2));
  std::cout << "X: " << x_ << std::endl;
  //std::cout << H_ << std::endl;
  return 1;
}  

void SecondOrderMinimizationSE2::update(
    const Sophus::SE2& old_model,
    Sophus::SE2& new_model)
{
  new_model = Sophus::SE2::exp(x_)*old_model;
  //std::cout << "new model " << new_model << std::endl;
}

void SecondOrderMinimizationSE2::startIteration()
{
  printf("Starting Iteratin\n");
}

void SecondOrderMinimizationSE2::finishIteration()
{
  printf("Finish Iteration\n");
}

} /* reloc */ 

#endif /* end of include guard: IMG_ALIGN_SE2_CPP */

