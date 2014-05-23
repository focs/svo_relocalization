
#include <svo_relocalization/yaml_parse.h>

#include <map>
#include <sstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

#include <svo/feature.h>
#include <vikit/atan_camera.h>

static char yaml_file_name[] = "frames_data.yaml";

void readYaml (std::string path, std::list<svo::FramePtr> &frames, vk::AbstractCamera *cam)
{

  std::cout << "Reading yaml fame file: " << path << std::endl;
  std::map<int,svo::Point*> tmp_id2point;

  std::ifstream fin (path+yaml_file_name);
  if (!fin)
  {
    std::cerr << "Error: can not read file" << std::endl;

  }

  YAML::Parser parser (fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
 
  // Points
  const YAML::Node *points_node = &doc["points"];

  for (size_t i = 0; i < points_node->size(); ++i)
  {
    Eigen::Vector3d pos;
    (*points_node)[i]["pos"][0] >> pos[0];
    (*points_node)[i]["pos"][1] >> pos[1];
    (*points_node)[i]["pos"][2] >> pos[2];

    int id;
    (*points_node)[i]["id"] >> id;

    svo::Point *p = new svo::Point(pos);
    p->id_ = id;

    tmp_id2point[id] = p;
  }

  // Frames
  const YAML::Node *frames_node = &doc["frames"];

  for (size_t i = 0; i < frames_node->size(); ++i)
  {
    // id
    int id;
    (*frames_node)[i]["id"] >> id;

    // pose
    double x, y, z;
    double rw, rx, ry, rz;
    (*frames_node)[i]["pose"][0] >> x;      
    (*frames_node)[i]["pose"][1] >> y;      
    (*frames_node)[i]["pose"][2] >> z;      
    (*frames_node)[i]["pose"][3] >> rx;      
    (*frames_node)[i]["pose"][4] >> ry;      
    (*frames_node)[i]["pose"][5] >> rz;
    (*frames_node)[i]["pose"][6] >> rw;      

    Eigen::Vector3d t(x,y,z);
    Eigen::Quaterniond q(rw, rx, ry, rz);
    Sophus::SE3 T_f_w (q, t);

    // image
    cv::Mat img;
    std::string img_name;
    (*frames_node)[i]["file_name"] >> img_name;
    std::cout << "Reading image " << path+img_name << std::endl;
    img = cv::imread(path+img_name, CV_LOAD_IMAGE_GRAYSCALE);

    // Create frame
    svo::FramePtr frame (new svo::Frame(cam, img, 0));
    frame->id_ = id;
    frame->T_f_w_ = T_f_w;
    frames.push_back(frame);

    for (size_t j = 0; j < (*frames_node)[i]["features"].size(); ++j)
    {
      Eigen::Vector2d px;
      (*frames_node)[i]["features"][j]["px"][0] >> px[0];      
      (*frames_node)[i]["features"][j]["px"][1] >> px[1];      

      Eigen::Vector3d f;
      (*frames_node)[i]["features"][j]["f"][0] >> f[0];      
      (*frames_node)[i]["features"][j]["f"][1] >> f[1];      
      (*frames_node)[i]["features"][j]["f"][2] >> f[2];      

      int pyr_lvl;
      (*frames_node)[i]["features"][j]["pyr_lvl"] >> pyr_lvl;      

      int point_id;
      svo::Point *p = NULL;
      (*frames_node)[i]["features"][j]["point"] >> point_id;      
      if (point_id >= 0)
        p = tmp_id2point[point_id];

      svo::Feature *feature  = new svo::Feature(frame.get(), p, px, f, pyr_lvl);
      frame->fts_.push_back(feature);
    }
  }

}

void writeYaml (std::string path, const std::list<svo::FramePtr> &frames, bool write_images)
{

  std::cout << "Writing " << frames.size() << " frames to " << path << std::endl;
  std::list<svo::Point*> points;

  YAML::Emitter out;
  // Veguin map with key frames
  out << YAML::BeginMap;
  out << YAML::Key << "frames";
  out << YAML::Value;
  // List of frames
  out << YAML::BeginSeq;
  for (auto it_frame = frames.begin();it_frame != frames.end(); ++it_frame)
  {
    out << YAML::BeginMap;

    // id
    out << YAML::Key << "id" << YAML::Value << (*it_frame)->id_;

    // pose
    Sophus::SE3 T_f_w = (*it_frame)->T_f_w_;
    Eigen::Vector3d t = T_f_w.translation();
    Sophus::Quaterniond r = T_f_w.unit_quaternion();
    out << YAML::Key << "pose";
    out << YAML::Value;
    out << YAML::Flow << YAML::BeginSeq;
    out << t[0] << t[1] << t[2];
    out << r.x() << r.y() << r.z() << r.w();
    out << YAML::EndSeq;
    
    std::stringstream ss;
    ss << "image" << std::setfill('0') << std::setw(5) << (*it_frame)->id_ << ".png";
    if (write_images)
      cv::imwrite(path+ss.str(), (*it_frame)->img_pyr_.at(0));
    // Image name
    out << YAML::Key << "file_name" << YAML::Value << ss.str();

    // write features
    out << YAML::Key << "features";
    out << YAML::Value;
    out << YAML::BeginSeq;
    for (auto it_feature = (*it_frame)->fts_.begin();
        it_feature != (*it_frame)->fts_.end();
        ++it_feature)
    {
      out << YAML::BeginMap;

      out << YAML::Key << "px";
      out << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq;
      // X
      out << (*it_feature)->px[0];
      // Y
      out << (*it_feature)->px[1];
      out << YAML::EndSeq;

      out << YAML::Key << "f";
      out << YAML::Value;
      out << YAML::Flow << YAML::BeginSeq;
      out << (*it_feature)->f[0];
      out << (*it_feature)->f[1];
      out << (*it_feature)->f[2];
      out << YAML::EndSeq;

      out << YAML::Key << "pyr_lvl";
      out << YAML::Value;
      // pyr level
      out << (*it_feature)->level;

      int point_id = -1;
      if ((*it_feature)->point != NULL)
      {
        point_id = (*it_feature)->point->id_;
        points.push_back((*it_feature)->point);
      }

      out << YAML::Key << "point";
      out << YAML::Value;
      // point id
      out << point_id;
      
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

  }
  // End list of frames
  out << YAML::EndSeq;

  out << YAML::Key << "points";
  out << YAML::Value;
  // List of points
  out << YAML::BeginSeq;
  for (auto it_point = points.begin(); it_point != points.end(); ++it_point)
  {
    out << YAML::BeginMap;

    // id
    out << YAML::Key << "id" << YAML::Value << (*it_point)->id_;
    // pos
    out << YAML::Key << "pos";
    out << YAML::Value;
    out << YAML::Flow << YAML::BeginSeq;
    out << (*it_point)->pos_[0];
    out << (*it_point)->pos_[1];
    out << (*it_point)->pos_[2];
    out << YAML::EndSeq;

    out << YAML::EndMap;

  }
  // End list of points
  out << YAML::EndSeq;
 
  out << YAML::EndMap;
  
  std::ofstream fout(path+yaml_file_name);
  fout << out.c_str();
  fout.close();

}

static char yaml_cam_model[] = "cam_model";
static char yaml_cam_width[] = "cam_width";
static char yaml_cam_height[] = "cam_height";
static char yaml_cam_fx[] = "cam_fx";
static char yaml_cam_fy[] = "cam_fy";
static char yaml_cam_cx[] = "cam_cx";
static char yaml_cam_cy[] = "cam_cy";
static char yaml_cam_d0[] = "cam_d0";
static char yaml_cam_calib_date[] = "cam_calib_date";
static char yaml_cam_name[] = "cam_name";
static char yaml_cam_description[] = "cam_description";

vk::AbstractCamera* readCameraYaml (std::string file)
{

  std::ifstream fin (file);

  YAML::Parser parser (fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  std::string tmp;
  doc[yaml_cam_model] >> tmp;

  if (tmp != std::string("ATAN"))
  {
    std::cout << "Error: not using ATAN model" << std::endl;
    exit(-1);
  }

  doc[yaml_cam_description] >> tmp;
  std::cout << "Reading Camera " << tmp << std::endl;

  int width, height;
  double fx, fy, cx, cy, d0;

  doc[yaml_cam_width] >> width;
  doc[yaml_cam_height] >> height;
  doc[yaml_cam_fx] >> fx;
  doc[yaml_cam_fy] >> fy;
  doc[yaml_cam_cx] >> cx;
  doc[yaml_cam_cy] >> cy;
  doc[yaml_cam_d0] >> d0;

  vk::ATANCamera *cam = new vk::ATANCamera(width, height, fx, fy, cx, cy, d0);

  return cam;

}
