
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <vikit/atan_camera.h>
#include <svo_relocalization/multiple_relocalizer.h>
#include <svo_relocalization/cc_place_finder.h>
#include <svo_relocalization/naive_place_finder.h>
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/5pt_relpos_finder.h>
#include <svo_relocalization/3pt_relpos_finder.h>
#include <svo_relocalization/empty_relpos_finder.h>
#include <svo_relocalization/frame.h>

using namespace std;
using namespace Eigen;
using namespace reloc;
using namespace cv;


void createPyr (const cv::Mat &im, int num_lvl, vector<Mat> &pyr_out)
{
  pyr_out.push_back(im);

  for (size_t i = 1; i < num_lvl; ++i)
  {
    Size s (pyr_out[i-1].cols/2, pyr_out[i-1].rows/2);
    Mat resized;
    cv::resize(pyr_out[i-1], resized, s);
    pyr_out.push_back(resized);
  }

}

void readData (string path, vector<FrameSharedPtr> &data, map<int, FrameSharedPtr> &data_map)
{
  std::ifstream file_in(path + string("pose.txt"));
  string line;
  int id=0;
  float x, y, z;
  float rw, rx, ry, rz;

  while (!file_in.eof())
  {
    file_in >> id >> x >> y >> z >>  rx >> ry >> rz >> rw;
    
    stringstream filename;
    filename << path << "image" << id << ".png";

    cv::Mat im;
    im = cv::imread(filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::flip(im,im, -1);

    Vector3d t(x,y,z);
    Eigen::Quaterniond q(rw, rx, ry, rz);
    Sophus::SE3 T_frame_world (q, t);

    FrameSharedPtr f (new Frame());

    createPyr(im, 4, f->img_pyr_);
    f->T_frame_world_ = T_frame_world;
    f->id_ = id;

    data.push_back(f);
    data_map[f->id_] = f;
  }

  file_in.close();
}

int main(int argc, char const *argv[])
{

  string training_images_path  = argv[1];
  string testing_images_path = argv[2];
  string results_path = argv[3];

  string place_finder_type = argv[4];
  string relpos_finder_type = argv[5];

  // Camera intrinsic parameters 
  float cam_width = std::stof(argv[6]);
  float cam_height = std::stof(argv[7]);
  Vector2d cam_size (cam_width, cam_height);
  float cam_fx = std::stof(argv[ 8]);
  float cam_fy = std::stof(argv[ 9]);
  float cam_cx = std::stof(argv[10]);
  float cam_cy = std::stof(argv[11]);
  float cam_d0 = std::stof(argv[12]);
  vk::ATANCamera my_camera (cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy, cam_d0); 

  AbstractPlaceFinderSharedPtr place_finder;
  if (place_finder_type == string("CC"))
  {
    place_finder = AbstractPlaceFinderSharedPtr (new CCPlaceFinder());
  } else if (place_finder_type == string("naive"))
  {
    place_finder = AbstractPlaceFinderSharedPtr (new NaivePlaceFinder());
  } else
  {
    cerr << "Place Finder type not found" << endl;
    exit(-1);
  }

  AbstractRelposFinderSharedPtr relpos_finder;
  if (relpos_finder_type == string("ESM"))
  {
    ESMRelposFinder *esm_pf = new ESMRelposFinder(&my_camera);
    esm_pf->options_.pyr_lvl_ = 3;
    relpos_finder = AbstractRelposFinderSharedPtr (esm_pf);
  } else if (relpos_finder_type == string("5pt"))
  {
    FivePtRelposFinder *five_pf = new FivePtRelposFinder(&my_camera);
    five_pf->options_.pyr_lvl_ = 3;

    relpos_finder = AbstractRelposFinderSharedPtr (five_pf);

  } else if (relpos_finder_type == string("3pt"))
  {
    relpos_finder = AbstractRelposFinderSharedPtr (new ThreePtRelposFinder(&my_camera));
  } else if (relpos_finder_type == string("empty"))
  {
    relpos_finder = AbstractRelposFinderSharedPtr (new EmptyRelposFinder());
  } else
  {
    cerr << "Relpos Finder type not found" << endl;
    exit(-1);
  }
  MultipleRelocalizer relocalizer(place_finder, relpos_finder);

  // Structures to hold data
  vector<FrameSharedPtr> training_data;
  vector<FrameSharedPtr> testing_data;
  map<int, FrameSharedPtr> testing_data_map;
  map<int, FrameSharedPtr> training_data_map;

  readData(training_images_path, training_data, training_data_map);
  readData(testing_images_path, testing_data, testing_data_map);

  cout << "Number training images: " << training_data.size() << endl;
  cout << "Number test images: " << testing_data.size() << endl;
  cout << "Number training images: " << training_data_map.size() << endl;
  cout << "Number test images: " << testing_data_map.size() << endl;
  

  for (size_t i = 0; i < training_data.size(); i+=1)
  {
    relocalizer.addFrame(training_data.at(i));
    //cout << "Addin training image id: " << training_data.at(i)->id_ << endl;
  }


  std::ofstream file_out (results_path + string("pose.txt"));
  for (size_t i = 0; i < testing_data.size(); ++i)
  {
    int id_out;
    Sophus::SE3 pose_out;
    relocalizer.relocalize(testing_data.at(i), pose_out, id_out);

    file_out << testing_data.at(i)->id_ << " " << 
      pose_out.translation()[0] << " " <<
      pose_out.translation()[1] << " " <<
      pose_out.translation()[2] << " " <<
      pose_out.unit_quaternion().x() << " " <<
      pose_out.unit_quaternion().y() << " " <<
      pose_out.unit_quaternion().z() << " " <<
      pose_out.unit_quaternion().w() << endl;
    //namedWindow( "Test img", WINDOW_AUTOSIZE );
    //imshow( "Test img", testing_data.at(i)->img_pyr_.at(0));
    //namedWindow( "Found img", WINDOW_AUTOSIZE );
    //imshow( "Found img", training_data_map[id_out]->img_pyr_.at(0));
    //waitKey(0);          
  }
  file_out.close();
  


  return 0;
}
