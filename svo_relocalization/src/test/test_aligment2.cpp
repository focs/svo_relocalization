
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se2.h>
#include <sophus/se3.h>
#include <fstream>
#include <vikit/atan_camera.h>

#include <svo_relocalization/img_aling_se2.h>
#include <svo_relocalization/esm_relpos_finder.h>

using namespace std;
using namespace cv;
using namespace Sophus;

void readMatrixFromFile (vector<string>& image_names, vector<Sophus::SE3>& poses, vector<int>& ids)
{
  std::ifstream file_in("/home/fox/catkin_ws/src/svo_relocalization/test_data/pose.txt");

  string path ("/home/fox/catkin_ws/src/svo_relocalization/test_data/");
  string line;
  int id=0;
  float x, y, z;
  float rw, rx, ry, rz;

  while (!file_in.eof())
  //for (size_t i = 0; i < 100; ++i)
  {
    file_in >> id >> x >> y >> z >>  rx >> ry >> rz >> rw;
    
    stringstream filename;
    filename << path << "image" << id << ".png";
    image_names.push_back(filename.str());

    Vector3d t(x,y,z);
    Eigen::Quaterniond q(rw, rx, ry, rz);
    poses.push_back(Sophus::SE3(q, t));

    ids.push_back(id);
  }

  file_in.close();
}


int main(int argc, char const *argv[])
{
  

  float cam_width = 752;
  float cam_height = 480;
  float cam_fx = 0.582533;
  float cam_fy = 0.910057;
  float cam_cx = 0.510927;
  float cam_cy = 0.526193;
  float cam_d0 = 0.916379;
  vk::ATANCamera my_camera (cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, cam_d0); 

  vector<string> image_names;
  vector<Sophus::SE3> poses;
  vector<int> ids;

  readMatrixFromFile(image_names, poses, ids);
  
  int template_idx = 50;
  int query_idx = template_idx + 1;
  Mat img_template = cv::imread(image_names.at(template_idx));
  Mat img_query = imread(image_names.at(query_idx));

  // Model from query to template
  SE2 se2_T_template_query;
  reloc::SecondOrderMinimizationSE2 minimizer(img_query, img_template);
  minimizer.optimizeGaussNewton(se2_T_template_query);

  cout << se2_T_template_query << endl;

  // convert found se2 image aligment ot a world rotation SO3
  SE3 se3_T_template_query;
  se3_T_template_query = reloc::ESMRelposFinder::findSE3(se2_T_template_query, &my_camera, 3);

  cout << "Found se3:" << endl << se3_T_template_query << endl;
  cout << "Query:" << endl << poses.at(query_idx) << endl;

  SE3 found_se3_T_template_world;
  found_se3_T_template_world = se3_T_template_query * poses.at(query_idx);
  cout << "Final found world to template:" << endl << found_se3_T_template_world << endl;
  cout << "Real world to template:" << endl << poses.at(template_idx) << endl;

  Vector2d center (my_camera.width()/2, my_camera.height()/2);
  Vector3d p (0.05,0.5,1);
  //p = my_camera.cam2world(Vector2d(5,0) + center);

  //cout << "With nosing" << endl << my_camera.world2cam( p ) << endl; 
  //cout << "With se2" << endl << se2_T_template_query*(my_camera.world2cam(p) - center) << endl;
  //cout << "With se3" << endl << my_camera.world2cam(se3_T_template_query * p ) - center << endl; 
  //cout << "difference wis nosin" << endl << my_camera.world2cam( p ) - center - se2_T_template_query*(my_camera.world2cam(p) - center) << endl;
  //cout << "difference" << endl << my_camera.world2cam(se3_T_template_query * p ) - center - 
  //se2_T_template_query*(my_camera.world2cam(p) - center) << endl;
  

  cout << "templat" << endl << poses.at(template_idx).matrix() << endl;
  cout << "query" << endl << poses.at(query_idx).matrix() << endl;
  cout << "Template throuth query" << endl << found_se3_T_template_world.matrix() << endl;
  cout << "found transformation" << endl << se3_T_template_query.matrix() << endl;
 


  return 0;
}
