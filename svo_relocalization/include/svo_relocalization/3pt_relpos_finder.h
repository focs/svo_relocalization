
#ifndef SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ
#define SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ


#include <svo_relocalization/abstract_relpos_finder.h>


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

  ThreePtRelposFinder ();
  virtual ~ThreePtRelposFinder ();

  Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match);

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ */

