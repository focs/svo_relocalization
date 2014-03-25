

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

#include <svo_relocalization/klein_murray_relocalizter.h>


using namespace std;

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
  TestConMatKMRelocalizer (string images_path, string suffix = ".png");
  virtual ~TestConMatKMRelocalizer (){};

  void startTest();
private:

  void loadImages();
  string images_path_;
  string suffix_;

  reloc::KMRelocalizer r;
};

TestConMatKMRelocalizer::TestConMatKMRelocalizer(string images_path, string suffix)
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

  cout << (images_path_+"confusion_matrix.txt") << endl;

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
  
  string folder = "/home/fox/catkin_ws/src/svo_relocalization/dense_input_data/";
  string suffix = ".png";
  
  TestConMatKMRelocalizer tester (folder);
  tester.startTest();

  return 0;
}
