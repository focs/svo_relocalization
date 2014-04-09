

#ifndef SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT
#define SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT

#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class FivePtRelposFinder : public AbstractRelposFinder
{
public:
  FivePtRelposFinder ();
  virtual ~FivePtRelposFinder ();

  Sophus::SE3 findRelpos(
      const FrameSharedPtr& frame_query,
      const FrameSharedPtr& frame_best_match);

private:

};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT */
