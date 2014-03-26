

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <stdio.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <vikit/atan_camera.h>

#include <svo_relocalization/klein_murray_relocalizter.h>


using namespace std;
using namespace Eigen;

bool has_suffix(const std::string &str, const std::string &suffix)
{
  return str.size() >= suffix.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}


vector<string> getFilesInFolder(const string& folder, const string& suffix)
{
  DIR *dir;
  struct dirent *de;

  vector<string> file_list;

  dir = opendir(folder.c_str()); /*your directory*/
  while(dir)
  {
    string file;
    de = readdir(dir);
    if (!de) break;
    //printf("%i %s\n", de->d_type, de->d_name);

    file = de->d_name;
    if (has_suffix(file, suffix))
      file_list.push_back(file);
  }
  closedir(dir);

  std::sort(file_list.begin(), file_list.end());

  return file_list;
}

class TestConMatKMRelocalizer
{
public:
  TestConMatKMRelocalizer (vk::ATANCamera *camera_model, string images_path, string suffix = ".png");
  virtual ~TestConMatKMRelocalizer (){};

  void startTest();
private:

  void loadImages();
  string images_path_;
  string suffix_;

  reloc::KMRelocalizer r;
};

TestConMatKMRelocalizer::TestConMatKMRelocalizer(vk::ATANCamera *camera_model, string images_path, string suffix) :
  r(camera_model)
{
 images_path_= images_path;
  suffix_= suffix;
}

void TestConMatKMRelocalizer::loadImages()
{
  vector<string> file_list;
  // Get a list of files ending with .png in the given folder
  file_list = getFilesInFolder(images_path_, suffix_);

  for (size_t i = 0; i < file_list.size(); ++i)
  {
    cv::Mat im = cv::imread(images_path_+file_list[i], CV_LOAD_IMAGE_GRAYSCALE);
    vector<cv::Mat> tmp_vec;
    tmp_vec.push_back(im);
    r.addFrame(tmp_vec, Sophus::SE3(), i);
  }
}

void TestConMatKMRelocalizer::startTest()
{
  loadImages();

  
  std::ofstream file;
  file.open((images_path_+"confusion_matrix.txt").c_str());


  std::list<reloc::KMRelocalizer::ImagePoseId>::iterator image_it, image_it_nested;

  for (image_it = r.images_.begin(); image_it != r.images_.end(); image_it++)
  {
    for (image_it_nested = r.images_.begin(); image_it_nested != r.images_.end(); image_it_nested++)
    {
      cv::Mat diff_im = image_it->image - image_it_nested->image;
      file << cv::sum(diff_im.mul(diff_im))[0] << ' ';
    }

    file << endl;
  }

  file.close();
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

  string folder;
  string suffix = ".png";

  if (argc < 2)
  {
    printf("Usage: %s <dataset path> [<image suffix>]\n", argv[0]);
    exit(0);
  }

  if (argc > 2)
  {
    suffix = string(argv[2]);
  }

  folder = string(argv[1]);
  
  
  TestConMatKMRelocalizer tester (&my_camera, folder);
  tester.startTest();

  return 0;
}
