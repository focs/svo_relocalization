
#ifndef SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ
#define SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ


#include <svo_relocalization/abstract_relpos_finder.h>

#include <vikit/abstract_camera.h>


namespace reloc
{

class ThreePtRelposFinder : public AbstractRelposFinder
{
public:

  struct Options {
    uint32_t pyr_lvl_;

    Options() :
      pyr_lvl_(3)
    {} 
  } options_;

  ThreePtRelposFinder (vk::AbstractCamera *camera_model);
  virtual ~ThreePtRelposFinder ();

  void removeFrame(int frame_id);

  void addFrame(const FrameSharedPtr &frame);

  Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match);

private:
  vk::AbstractCamera *camera_model_;

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ */

