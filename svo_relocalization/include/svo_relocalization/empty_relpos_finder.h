
#ifndef SVO_RELOCALIZER_EMPTY_RELPOS_FINDER_H_7TYZ8NMP
#define SVO_RELOCALIZER_EMPTY_RELPOS_FINDER_H_7TYZ8NMP


#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class EmptyRelposFinder : public AbstractRelposFinder
{
public:

  EmptyRelposFinder () {};
  virtual ~EmptyRelposFinder () {};

  void removeFrame(int frame_id) {};

  void addFrame(const FrameSharedPtr &frame) {};

  Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match)
  {
    return Sophus::SE3();
    
  };

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_EMPTY_RELPOS_FINDER_H_7TYZ8NMP */

