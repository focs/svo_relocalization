
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <vikit/atan_camera.h>
#include <svo_relocalization/multiple_relocalizer.h>

using namespace std;
using namespace Eigen;
using namespace reloc;


void readMatrixFromFile (vector<cv::Mat>& images, vector<Sophus::SE3>& poses, vector<int>& ids)
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
    cv::Mat im;
    im = cv::imread(filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
    images.push_back(im);

    Vector3d t(x,y,z);
    Eigen::Quaterniond q(rw, rx, ry, rz);
    poses.push_back(Sophus::SE3(q, t));

    ids.push_back(id);
  }

  file_in.close();
}

int main(int argc, char const *argv[])
{
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

  vector<cv::Mat> images;
  vector<Sophus::SE3> poses;
  vector<int> ids;

  readMatrixFromFile(images, poses, ids);
  cout << images.size() << endl;
  cout << "Image size: " << images[0].size() << endl;


  MultipleRelocalizer relocalizer(&my_camera);
  int query_idx = 253;

  for (size_t i = 0; i < images.size(); i+=1)
  {
    if (i != query_idx)
    {
      vector<cv::Mat> tmp_vector;
      tmp_vector.push_back(images.at(i));
      relocalizer.addFrame(tmp_vector, poses[i], ids[i]);
    }
  }

  cout << "Query image id: " << ids.at(query_idx) << endl;
  vector<cv::Mat> tmp_vector;
  tmp_vector.push_back(images.at(query_idx));
  Sophus::SE3 T_f_out;
  int id_out;

  relocalizer.relocalize(tmp_vector, poses.at(query_idx), T_f_out, id_out);


  return 0;
}
