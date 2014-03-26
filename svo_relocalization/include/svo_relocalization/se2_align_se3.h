
#ifndef SVO_RELOCALIZATION_E2_ALIGN_SE3_H_D1I8XULS
#define SVO_RELOCALIZATION_E2_ALIGN_SE3_H_D1I8XULS

#include <Eigen/Core>

#include <vikit/nlls_solver.h>
#include <vikit/abstract_camera.h>
#include <sophus/se2.h> 
#include <sophus/so3.h>

namespace reloc
{

class SE2toSE3 : public vk::NLLSSolver <3,Sophus::SE3>
{

private:
  typedef vk::AbstractCamera LocalCameraModel;

public:
  SE2toSE3 (const Sophus::SE2& SE2_model, LocalCameraModel *camera_model);
  virtual ~SE2toSE3 () {};

protected:
  virtual double
  computeResiduals (
      const Sophus::SE3& model,
      bool linearize_system,
      bool compute_weight_scale = false);

  virtual int
  solve();

  virtual void
  update(const Sophus::SE3& old_model,  Sophus::SE3& new_model);

private:
  LocalCameraModel *camera_model_;
  Eigen::Vector2d px_trans_ [2]; //! pixel points with SE2 transform
  Eigen::Vector3d uv_ [2]; //! px points on image plane coordinates (no transforma)

};
  
} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZATION_E2_ALIGN_SE3_H_D1I8XULS */

