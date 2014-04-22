
#ifndef SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2
#define SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2

#include <svo_relocalization/abstract_relpos_finder.h>

#include <vikit/abstract_camera.h>
#include <sophus/se2.h>

namespace reloc
{

class ESMRelposFinder : public AbstractRelposFinder
{
public:
  struct Options {
    uint32_t pyr_lvl_;
    uint32_t n_iter_se2_to_se3_;

    Options() :
      pyr_lvl_(3),
      n_iter_se2_to_se3_(3)
    {} 
  } options_;

  ESMRelposFinder(vk::AbstractCamera *camera_model);
  virtual ~ESMRelposFinder();

  void removeFrame(int frame_id);
  void addFrame(const FrameSharedPtr &frame);

  Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match);

  static Sophus::SE3 findSE3(
      Sophus::SE2 t,
      vk::AbstractCamera *camera_model,
      uint32_t n_iter);

private:
  vk::AbstractCamera *camera_model_;

};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2 */

