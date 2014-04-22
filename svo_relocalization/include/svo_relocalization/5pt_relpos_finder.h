

#ifndef SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT
#define SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT

#include <svo_relocalization/abstract_relpos_finder.h>

#include <vikit/abstract_camera.h>


namespace reloc
{

class FivePtRelposFinder : public AbstractRelposFinder
{
public:
  struct Options {
    uint32_t pyr_lvl_;

    Options() :
      pyr_lvl_(3)
    {} 
  } options_;

  FivePtRelposFinder (vk::AbstractCamera *camera_model);
  virtual ~FivePtRelposFinder ();

  void removeFrame(int frame_id);

  void addFrame(const FrameSharedPtr &frame);

  Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match);

private:

  vk::AbstractCamera *camera_model_;

};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT */
