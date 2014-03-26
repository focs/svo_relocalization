
#include <svo_relocalization/se2_align_se3.h> 

namespace reloc
{
  
SE2toSE3::SE2toSE3 (const Sophus::SE2& SE2_model, vk::AbstractCamera *camera_model) :
  camera_model_(camera_model)
{

  Eigen::Vector2d cam_size (camera_model_->width(), camera_model_->height());
  Eigen::Vector2d px [2];
  px[0] << 5, 0;
  px[1] << -5, 0;

  // Apply transform
  px_trans_[0] = cam_size/2 + SE2_model * px[0];
  px_trans_[1] = cam_size/2 + SE2_model * px[1];

  // Get image frame coords
  uv_[0] = camera_model_->cam2world(cam_size/2 + px[0]);
  uv_[1] = camera_model_->cam2world(cam_size/2 + px[1]);
}

double SE2toSE3::computeResiduals (
    const Sophus::SE3& model,
    bool linearize_system,
    bool compute_weight_scale)
{

  typedef Eigen::Matrix<double,2,3> Matrix23d;

  H_.setZero();
  Jres_.setZero();

  Sophus::SO3 rotation_model = model.so3();
  Eigen::Vector2d px_error;
  // For the two points
  for (size_t p_idx = 0; p_idx < 2; ++p_idx)
  {
    Eigen::Vector3d uv_rot;
    uv_rot = rotation_model*uv_[p_idx];

    Eigen::Vector2d px_rot;
    px_rot = camera_model_->world2cam(uv_rot);

    px_error = px_trans_[p_idx] - px_rot;

    if (linearize_system)
    {
      // Derivative of \pi
      Matrix23d d_pi;
      d_pi << 1,0,-uv_rot[0]/uv_rot[2],
           0,1,-uv_rot[1]/uv_rot[2];

      Eigen::Matrix2d focal_mat;
      focal_mat.setZero();
      // Focal divided by z
      focal_mat.diagonal().setConstant(camera_model_->errorMultiplier2()/uv_rot[2]);
      d_pi = focal_mat * d_pi;

      Matrix23d Jac;
      // For the three generators
      for (size_t g_idx = 0; g_idx < 3; ++g_idx)
      {
        // Jacobian
        Jac.col(g_idx) = -d_pi * Sophus::SO3().generator(g_idx)*uv_rot;

        H_ += Jac.transpose() * Jac;
        Jres_ += Jac.transpose() * px_error;
      }
    }
  }

  return px_error.norm();
}

int SE2toSE3::solve()
{
  x_ = H_.ldlt().solve(-Jres_);

  if((bool) std::isnan((double) x_[0]))
    return 0;
  return 1;
}

void SE2toSE3::update(const Sophus::SE3& old_model,  Sophus::SE3& new_model)
{
  Sophus::SE3 tmp;
  tmp.setQuaternion(Sophus::SO3::exp(x_).unit_quaternion());
    new_model = tmp * old_model;
}

} /* reloc */ 
