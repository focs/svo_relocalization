
#ifndef SVO_RELOCALIZATION_IMG_ALING_SE2_H
#define SVO_RELOCALIZATION_IMG_ALING_SE2_H

#include <opencv2/opencv.hpp>
#include <sophus/se2.h>
#include <vikit/nlls_solver.h>

namespace reloc
{
  
class SecondOrderMinimizationSE2 : public vk::NLLSSolver<3,Sophus::SE2>
{
public:
  SecondOrderMinimizationSE2 (const cv::Mat& im, const cv::Mat& im_template);
  virtual ~SecondOrderMinimizationSE2 ();


protected:
  virtual double
  computeResiduals (
      const Sophus::SE2& model,
      bool linearize_system,
      bool compute_weight_scale = false);

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
  cv::Mat im_;
  cv::Mat im_template_;
  cv::Mat im_template_grad_x_;
  cv::Mat im_template_grad_y_;

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZATION_IMG_ALING_SE2_H */

