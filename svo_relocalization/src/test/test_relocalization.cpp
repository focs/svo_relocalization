
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <vikit/atan_camera.h>
#include <svo_relocalization/multiple_relocalizer.h>
#include <svo_relocalization/cc_place_finder.h>
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/frame.h>

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


  AbstractPlaceFinderSharedPtr cc_shared (new CCPlaceFinder());
  AbstractRelposFinderSharedPtr esm_shared (new ESMRelposFinder(&my_camera));
  MultipleRelocalizer relocalizer(cc_shared, esm_shared);

  int query_idx = 253;
  for (size_t i = 0; i < images.size(); i+=1)
  {
    if (i != static_cast<size_t>(query_idx))
    {
      // Create and fill frame
      FrameSharedPtr frame_shared (new Frame());
      frame_shared->img_pyr_.push_back(images.at(i));
      frame_shared->T_frame_world_ = poses.at(i);
      frame_shared->id_ = ids.at(i);
      
      relocalizer.addFrame(frame_shared);
    }
  }

  cout << "Query image id: " << ids.at(query_idx) << endl;
  int id_out;
  FrameSharedPtr frame_shared (new Frame());
  frame_shared->img_pyr_.push_back(images.at(query_idx));

  relocalizer.relocalize(frame_shared, id_out);


  return 0;
}
