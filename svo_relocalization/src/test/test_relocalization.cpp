
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
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
#include <svo_relocalization/yaml_parse.h>
#include <svo_relocalization/svo_reloc_utils.h>
#include <svo_relocalization/ferns_relocalizer.h>

#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>

using namespace std;
using namespace Eigen;
using namespace reloc;
using namespace cv;

static char yaml_calib_file[] = "calib_file";
static char yaml_train_path[] = "train_path";
static char yaml_test_path[] = "test_path";
static char yaml_results_path[] = "results_path";
static char yaml_place_finder[] = "place_finder";
static char yaml_relpos_finder[] = "relpos_finder";
static char yaml_method[] = "method";
static char yaml_num_ferns [] = "num_ferns";
static char yaml_ferns_num_tests [] = "ferns_num_tests";
static char yaml_ferns_patch_size [] = "ferns_patch_size";

//void createPyr (const cv::Mat &im, int num_lvl, vector<Mat> &pyr_out)
//{
//  pyr_out.push_back(im);
//
//  for (size_t i = 1; i < num_lvl; ++i)
//  {
//    Size s (pyr_out[i-1].cols/2, pyr_out[i-1].rows/2);
//    Mat resized;
//    cv::resize(pyr_out[i-1], resized, s);
//    pyr_out.push_back(resized);
//  }
//
//}
//
//void readData (string path, vector<FrameSharedPtr> &data, map<int, FrameSharedPtr> &data_map)
//{
//  std::ifstream file_in(path + string("pose.txt"));
//  string line;
//  int id=0;
//  float x, y, z;
//  float rw, rx, ry, rz;
//
//  while (!file_in.eof())
//  {
//    file_in >> id >> x >> y >> z >>  rx >> ry >> rz >> rw;
//    
//    stringstream filename;
//    filename << path << "image" << id << ".png";
//
//    cv::Mat im;
//    im = cv::imread(filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
//    cv::flip(im,im, -1);
//
//    Vector3d t(x,y,z);
//    Eigen::Quaterniond q(rw, rx, ry, rz);
//    Sophus::SE3 T_frame_world (q, t);
//
//    FrameSharedPtr f (new Frame());
//
//    createPyr(im, 4, f->img_pyr_);
//    f->T_frame_world_ = T_frame_world;
//    f->id_ = id;
//
//    data.push_back(f);
//    data_map[f->id_] = f;
//  }
//
//  file_in.close();
//}


void readParams (string path, map<string,string> &params)
{

  std::ifstream fin (path);

  YAML::Parser parser (fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  string tmp;
  doc[yaml_calib_file] >> tmp;
  params[yaml_calib_file] = tmp;

  doc[yaml_calib_file] >> tmp;
  params[yaml_calib_file] = tmp;

  doc[yaml_train_path] >> tmp;
  params[yaml_train_path] = tmp;

  doc[yaml_test_path] >> tmp;
  params[yaml_test_path] = tmp;

  doc[yaml_results_path] >> tmp;
  params[yaml_results_path] = tmp;

  doc[yaml_place_finder] >> tmp;
  params[yaml_place_finder] = tmp;

  doc[yaml_relpos_finder] >> tmp;
  params[yaml_relpos_finder] = tmp;

  doc[yaml_method] >> tmp;
  params[yaml_method] = tmp;

  doc[yaml_num_ferns] >> tmp;
  params[yaml_num_ferns] = tmp;

  doc[yaml_ferns_num_tests] >> tmp;
  params[yaml_ferns_num_tests] = tmp;

  doc[yaml_ferns_patch_size] >> tmp;
  params[yaml_ferns_patch_size] = tmp;
}


int main(int argc, char const *argv[])
{
  clock_t tic, toc;
  double time_diff, time_total;

  if (argc < 2)
  {
    cout << "Usange: " << argv[0] << " <param file>" << endl;
    exit(-1);
  }

  map<string,string> params;
  readParams(argv[1], params);
  
  cout << "Num params " << params.size() << endl;

  for (auto t : params)
    std::cout << t.first << " -> " << t.second << endl;

  vk::AbstractCamera *cam = readCameraYaml(params[yaml_calib_file]);


  AbstractPlaceFinderSharedPtr place_finder;
  if (params[yaml_place_finder] == string("CC"))
  {
    place_finder = AbstractPlaceFinderSharedPtr (new CCPlaceFinder());
  } else if (params[yaml_place_finder] == string("naive"))
  {
    place_finder = AbstractPlaceFinderSharedPtr (new NaivePlaceFinder());
  } else
  {
    cerr << "Place Finder type not found" << endl;
    exit(-1);
  }

  AbstractRelposFinderSharedPtr relpos_finder;
  if (params[yaml_relpos_finder] == string("ESM"))
  {
    ESMRelposFinder *esm_pf = new ESMRelposFinder(cam);
    esm_pf->options_.pyr_lvl_ = 3;
    relpos_finder = AbstractRelposFinderSharedPtr (esm_pf);
  } else if (params[yaml_relpos_finder] == string("5pt"))
  {
    FivePtRelposFinder *five_pf = new FivePtRelposFinder(cam);
    five_pf->options_.pyr_lvl_ = 2;

    relpos_finder = AbstractRelposFinderSharedPtr (five_pf);

  } else if (params[yaml_relpos_finder] == string("3pt"))
  {
    ThreePtRelposFinder *three_pf = new ThreePtRelposFinder(cam);
    three_pf->options_.pyr_lvl_ = 3;
    relpos_finder = AbstractRelposFinderSharedPtr (three_pf);

  } else if (params[yaml_relpos_finder] == string("empty"))
  {
    relpos_finder = AbstractRelposFinderSharedPtr (new EmptyRelposFinder());
  } else
  {
    cerr << "Relpos Finder type not found" << endl;
    exit(-1);
  }

  MultipleRelocalizer multi_relocalizer(place_finder, relpos_finder);
  FernsRelocalizer ferns_relocalizer(cam,
      stoi(params[yaml_num_ferns]),
      stoi(params[yaml_ferns_num_tests]),
      stoi(params[yaml_ferns_patch_size]),
      stoi(params[yaml_ferns_patch_size]));

  AbstractRelocalizer *relocalizer;
  if (params[yaml_method] == string("multi"))
    relocalizer = &multi_relocalizer;
  else if (params[yaml_method] == string("ferns"))
    relocalizer = &ferns_relocalizer;
  else
  {
    cerr << "Relocalizer method not found" << endl;
    exit(-1);
  }

  // Data!!!
  list<svo::FramePtr> train_frames;
  list<svo::FramePtr> test_frames;
  readYaml(params[yaml_train_path], train_frames, cam);
  readYaml(params[yaml_test_path], test_frames, cam);
  map<int,svo::FramePtr> svoframe_id2ptr;

  for(auto it_frame = train_frames.begin();
      it_frame != train_frames.end();
      ++it_frame)
  {
    svoframe_id2ptr[(*it_frame)->id_] = *it_frame;
    reloc::FrameSharedPtr reloc_frame (new reloc::Frame());
    svo2reloc(*it_frame, reloc_frame);
    
    relocalizer->addFrame(reloc_frame);
  }

  cout << "Done adding frames" << endl;

  //Train
  tic = clock();
  relocalizer->train();
  toc = clock();
  time_diff = static_cast<double>(toc - tic)/CLOCKS_PER_SEC;

  cout << "Time to train " << time_diff << " sec" << endl;

  cout << "Done training" << endl;
  
  time_total = 0;
  for(auto it_frame = test_frames.begin();
      it_frame != test_frames.end();
      ++it_frame)
  {
    reloc::FrameSharedPtr reloc_frame (new reloc::Frame());
    svo2reloc(*it_frame, reloc_frame);
    
    Sophus::SE3 T_q_f_out, found_T_f_w;
    int id_out;
    tic = clock();
    relocalizer->relocalize(reloc_frame, T_q_f_out, id_out);
    toc = clock();
    found_T_f_w = (T_q_f_out * svoframe_id2ptr[id_out]->T_f_w_);

    time_diff = static_cast<double>(toc - tic)/CLOCKS_PER_SEC;
    time_total += time_diff;

    cout << "Found id: " << id_out << endl;
    cout << "Closest pose: " << endl << svoframe_id2ptr[id_out]->T_f_w_ << endl;
    cout << "Found pose: " << endl << found_T_f_w << endl;
    //cout << "Found pose: " << endl << (T_f_w_out) << endl;
    cout << "Real pose: " << endl << reloc_frame->T_frame_world_;
    cout << "Error: " << (found_T_f_w.translation() - reloc_frame->T_frame_world_.translation()).norm() << endl << endl;

    cout << "Relocalize time " << time_diff << " sec" << endl;

    
    (*it_frame)->T_f_w_ = found_T_f_w;
  }

  cout << "Mean relocalization time " << time_total/test_frames.size() << " sec" << endl;

  writeYaml(params[yaml_results_path], test_frames, false);

  return 0;

}
