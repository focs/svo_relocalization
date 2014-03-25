
#include <iostream>

#include <sophus/se3.h>
#include <sophus/so3.h>
#include <sophus/se2.h>
#include <vikit/atan_camera.h>

using namespace std;
using namespace Eigen;
int main(int argc, char *argv[])
{
  typedef Matrix<double,2,3> Matrix23d;
  // set up generator matrices for SO3
  Matrix3d generators [3];
  generators[0].setZero();
  generators[1].setZero();
  generators[2].setZero();

  generators[0](1,2) = -1;
  generators[0](2,1) = 1;
  generators[1](0,2) = 1;
  generators[1](2,0) = -1;
  generators[2](0,1) = -1;
  generators[2](1,0) = 1;

  cout << generators[0] << endl;
  cout << generators[1] << endl;
  cout << generators[2] << endl;


  // Camera intrinsic parameters 
  float cam_width = 752;
  float cam_height = 480;
  Vector2d cam_size (cam_width, cam_height);
  float cam_fx = 0.582533;
  float cam_fy = 0.910057;
  float cam_cx = 0.510927;
  float cam_cy = 0.526193;
  float cam_d0 = 0.916379;

  vk::ATANCamera my_camera (cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, cam_d0);

  // Test points
  Vector2d px [2], px_trans[2];
  px[0] << 5, 0;
  px[1] << -5, 0;

  // SE2 found by "optimization"
  Sophus::SE2 found_T_template_query(0.02, Vector2d(5,0));
  px_trans[0] = cam_size/2 + found_T_template_query * px[0];
  px_trans[1] = cam_size/2 + found_T_template_query * px[1];
  cout << "px0: " << px_trans[0] << endl;
  cout << "px1: " << px_trans[1] << endl;

  Vector3d uv [2];
  uv[0] = my_camera.cam2world(cam_size/2 + px[0]);
  uv[1] = my_camera.cam2world(cam_size/2 + px[1]);

  cout << uv[0] << endl;
  cout << uv[1] << endl;

  Sophus::SO3 rotation_model;
  cout << "Initial model:" << rotation_model << endl;
  // i is only the number of iterations
  for (size_t i = 0; i < 9; ++i)
  {
    Matrix3d Hess;
    Vector3d Jd;
    Hess.setZero();
    Jd.setZero();
    // For the two points
    for (size_t p_idx = 0; p_idx < 2; ++p_idx)
    {
      Vector3d uv_rot;
      uv_rot = rotation_model*uv[p_idx];

      Vector2d px_rot;
      px_rot = my_camera.world2cam(uv_rot);

      Vector2d px_error;
      px_error = px_trans[p_idx] - px_rot;

      // Derivative of \pi
      Matrix23d d_pi;
      d_pi << 1,0,-uv_rot[0]/uv_rot[2],
           0,1,-uv_rot[1]/uv_rot[2];
      cout << d_pi << endl;

      Matrix2d focal_mat;
      focal_mat.setZero();
      // Focal divided by z
      focal_mat.diagonal() << my_camera.focal_length()/uv_rot[2];
      d_pi = focal_mat * d_pi;

      Matrix23d Jac;
      // For the three generators
      for (size_t g_idx = 0; g_idx < 3; ++g_idx)
      {
        // Jacobian
        Jac.col(g_idx) = -d_pi * Sophus::SO3().generator(g_idx)*uv_rot;

        Hess += Jac.transpose() * Jac;
        Jd += Jac.transpose() * px_error;
      }

      cout << "Error: " << px_error.norm() << endl;
    }

    Vector3d x;
    x = Hess.ldlt().solve(-Jd);
    rotation_model = Sophus::SO3::exp(x) * rotation_model;

    cout << rotation_model;

  } 




  return 0;
}
