
#include <svo_relocalization/yaml_parse.h>

#include <map>
#include <sstream>
#include <iomanip>
#include <yaml-cpp/yaml.h>

#include <svo/feature.h>

void readYaml (std::string path, std::list<svo::FramePtr> &frames, vk::AbstractCamera *cam)
{
  std::map<int,svo::Point*> tmp_id2point;

  std::ifstream fin ("/tmp/test.yaml");

  YAML::Parser parser (fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
 
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

  const YAML::Node *frames_node = &doc["frames"];

  for (size_t i = 0; i < frames_node->size(); ++i)
  {
    // id
    int id;
    (*frames_node)[i]["id"] >> id;

    // image
    cv::Mat img;
    std::string img_path;
    (*frames_node)[i]["file_name"] >> img_path;
    img = cv::imread(img_path);

    // Create frame
    svo::FramePtr frame (new svo::Frame(cam, img, 0));
    frame->id_ = id;

    for (size_t j = 0; j < (*frames_node)[i]["features"].size(); ++j)
    {
      int id;
      (*frames_node)[i]["features"]["id"] >> id;
      
      Eigen::Vector2d px;
      (*frames_node)[i]["features"]["px"][0] >> px[0];      
      (*frames_node)[i]["features"]["px"][1] >> px[2];      

      Eigen::Vector3d f;
      (*frames_node)[i]["features"]["f"][0] >> f[0];      
      (*frames_node)[i]["features"]["f"][1] >> f[1];      
      (*frames_node)[i]["features"]["f"][2] >> f[2];      

      int pyr_lvl;
      (*frames_node)[i]["features"]["pyr_lvl"] >> pyr_lvl;      

      int point_id;
      svo::Point *p = NULL;
      (*frames_node)[i]["features"]["point"] >> point_id;      
      if (point_id >= 0)
        p = tmp_id2point[point_id];

      svo::Feature *feature  = new svo::Feature(frame.get(), p, px, f, pyr_lvl);
      frame->fts_.push_back(feature);
    }

  }

}

void writeYaml (std::string path, const std::list<svo::FramePtr> &frames)
{
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
    
    std::stringstream ss;
    ss << "image" << std::setfill('0') << std::setw(5) << (*it_frame)->id_;
    cv::imwrite(path+ss.str()+".png", (*it_frame)->img_pyr_.at(0));
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
  
  std::ofstream fout(path+"data.yaml");
  fout << out.c_str();
  fout.close();

}

