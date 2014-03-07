
#ifndef IMG_ALIGN_SE2_CPP
#define IMG_ALIGN_SE2_CPP

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <svo_relocalization/img_aling_se2.h>


double SecondOrderMinimizationSE2::computeResiduals (
    const Sophus::SE2& model, 
    bool linearize_system, 
    bool compute_weight_scale)
{
  // Warp template image
  cv::Mat warp_mat(3, 3, CV_32F);
  cv::eigen2cv(model.matrix(), warp_mat);

  cv::Mat im_warp = cv::Mat::zeros(im.rows, im.cols, im.type());
  cv::warpAffine(im, im_warp, warp_mat, im_warp.size());

  // Compute warped image gradient
  cv::Mat im_warp_grad_x;
  cv::Mat im_warp_grad_y;
  cv::Sobel(im_warp, im_warp_grad_x, CV_32F, 1, 0, 1);
  cv::Sobel(im_warp, im_warp_grad_y, CV_32F, 0, 1, 1);

  // Compute mean grad
  cv::Mat im_grad_mean_x = 0.25*(im_template_grad_x + im_warp_grad_y);
  cv::Mat im_grad_mean_y = 0.25*(im_template_grad_y + im_warp_grad_y);

  // Compute error image
  cv::Mat im_error = im_warp - im_template;

  float chi2 = 0;
  for (size_t v = 0; v < im.rows; ++v)
  {
    for (size_t u = 0; u < im.cols; ++u)
    {
      // Jacovian/Steepest image
      // Angle ~ 0 so sin(0) = 0 and cos(0) = 1
      // J(2) = (-xsin(a) + ycos(a))*grad_x + (-xcos(a) - ysin(a))*grad_y
      Eigen::Vector3d J;
      J(0) = im_grad_mean_x.at<float>(v,u);
      J(1) = im_grad_mean_y.at<float>(v,u);
      J(2) = v*im_grad_mean_x.at<float>(u,v) - v*im_grad_mean_y.at<float>(v,u); 

      // Compute lower triangle of H
      for (size_t i = 0; i < J.size(); ++i)
      {
        for (size_t j = 0; j < i; ++j)
        {
          H_(i,j) = J(i) * J(j);
        }
      }

      // Compute Jres
      Jres_ += J*im_error.at<float>(u,v);

      // Compute total error
      chi2 += im_error.at<float>(u,v) * im_error.at<float>(u,v);
    }
  }

  // Fill uper triangle of H

  return chi2/(im.rows*im.cols);
  
}


int SecondOrderMinimizationSE2::solve()
{
  x_ = H_.ldlt().solve(-Jres_);
  return 1;
}  

void SecondOrderMinimizationSE2::update(
    const Sophus::SE2& old_model,
    Sophus::SE2& new_model)
{

}

void SecondOrderMinimizationSE2::startIteration()
{

}

void SecondOrderMinimizationSE2::finishIteration()
{

}

#endif /* end of include guard: IMG_ALIGN_SE2_CPP */

