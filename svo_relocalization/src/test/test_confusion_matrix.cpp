

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

#include <svo_relocalization/yaml_parse.h>
#include <svo_relocalization/cc_place_finder.h>


using namespace std;
using namespace Eigen;
using namespace cv;
using namespace reloc;

//bool has_suffix(const std::string &str, const std::string &suffix)
//{
//  return str.size() >= suffix.size() &&
//         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
//}
//
//
//vector<string> getFilesInFolder(const string& folder, const string& suffix)
//{
//  DIR *dir;
//  struct dirent *de;
//
//  vector<string> file_list;
//
//  dir = opendir(folder.c_str()); /*your directory*/
//  while(dir)
//  {
//    string file;
//    de = readdir(dir);
//    if (!de) break;
//    //printf("%i %s\n", de->d_type, de->d_name);
//
//    file = de->d_name;
//    if (has_suffix(file, suffix))
//      file_list.push_back(folder+"/"+file);
//  }
//  closedir(dir);
//
//  std::sort(file_list.begin(), file_list.end());
//
//  return file_list;
//}
//
//void loadImages(vector<string> &file_list, vector<Mat> &images_out)
//{
//  for (size_t i = 0; i < file_list.size(); ++i)
//  {
//    cv::Mat im = cv::imread(file_list.at(i), CV_LOAD_IMAGE_GRAYSCALE);
//    images_out.push_back(im);
//  }
//}

void startTest(vector<Mat> images, string path)
{
  
  std::ofstream file;
  file.open((path+"/confusion_matrix.txt").c_str());

  CCPlaceFinder r;
  for (size_t i = 0; i < images.size(); ++i)
  {
    FrameSharedPtr frame_shared (new Frame());
    frame_shared->img_pyr_.push_back(images.at(i));
    frame_shared->id_ = i;

    r.addFrame(frame_shared);
  }

  for (size_t i = 0; i < images.size(); ++i)
  {
    for (size_t j = 0; j < images.size(); ++j)
    {
      cv::Mat diff_im = r.getSmallBlurryImage(i)- r.getSmallBlurryImage(j);
      file << cv::sum(diff_im.mul(diff_im))[0] << ' ';
    }
    file << endl;
  }

}

int main(int argc, char const *argv[])
{

  string folder;
  list<svo::FramePtr> frames;
  vector<cv::Mat> images;
  vk::AbstractCamera *cam;
  
  if (argc < 2)
  {
    printf("Usage: %s <dataset path> \n", argv[0]);
    exit(0);
  }


  cam = readCameraYaml("/home/fox/catkin_ws/src/rpg_svo/svo_ros/param/calib/bluefox_847_cfo_atan.yaml");


  folder = string(argv[1]);
  
  readYaml(folder, frames, cam);

  for (auto it_frame = frames.begin();
      it_frame != frames.end();
      ++it_frame)
  {
    images.push_back((*it_frame)->img_pyr_.at(0));
  }

  startTest(images, folder);


  return 0;
}
