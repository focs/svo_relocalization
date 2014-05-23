
#ifndef SVO_RELOCALIZATION_FERNS_RELOCALIZER_H_LYFCPNM7
#define SVO_RELOCALIZATION_FERNS_RELOCALIZER_H_LYFCPNM7

#include <svo_relocalization/abstract_relocalizer.h>

#include <map>
#include <vector>

#include <svo_relocalization/frame.h>
#include <svo_relocalization/fern_classifier.h>

#include <vikit/abstract_camera.h>


namespace reloc
{

class FernsRelocalizer : public AbstractRelocalizer
{

public:
  FernsRelocalizer(
    vk::AbstractCamera *cam,
    int num_ferns,
    int num_tests,
    int max_x,
    int max_y);

  virtual ~FernsRelocalizer ();

  void removeFrame(int frame_id);
  
  void addFrame(FrameSharedPtr frame);

  void train();

  bool relocalize(
      FrameSharedPtr frame_query,
      Sophus::SE3 &pose_out,
      int &id_out);

private:

  FrameSharedPtr findClosestFrame(Sophus::SE3 pose);

  FernClassifier fern_classifier;
  std::vector<Eigen::Vector4i> train_data;
  std::vector<FrameSharedPtr> frames;

  int frame_counter;
  int point_counter;
  bool hasTrained;

  std::map<int,int> frame_idx2id;
  std::map<int,int> frame_id2idx;
  std::map<int,int> point_idx2id;
  std::map<int,int> point_id2idx;
  std::map<int,Eigen::Vector3d> point_id2pose;

  vk::AbstractCamera *cam_;
};

}

#endif /* end of include guard: SVO_RELOCALIZATION_FERNS_RELOCALIZER_H_LYFCPNM7 */

