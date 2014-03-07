
#ifndef IMG_ALING_SE2_H
#define IMG_ALING_SE2_H

#include <opencv2/opencv.hpp>
#include <sophus/se2.h>
#include <vikit/nlls_solver.h>

class SecondOrderMinimizationSE2 : public vk::NLLSSolver<3,Sophus::SE2>
{
public:
  SecondOrderMinimizationSE2 ();
  virtual ~SecondOrderMinimizationSE2 ();

protected:
  virtual double
  computeResiduals (const Sophus::SE2& model, bool linearize_system, bool compute_weight_scale = false);

  virtual int
  solve();

  virtual void
  update(const Sophus::SE2& old_model,  Sophus::SE2& new_model);

  virtual void
  startIteration();

  virtual void
  finishIteration();

private:
  /* data */
  cv::Mat im;
  cv::Mat im_template;
  cv::Mat im_template_grad_x;
  cv::Mat im_template_grad_y;

};


#endif /* end of include guard: IMG_ALING_SE2_H */

