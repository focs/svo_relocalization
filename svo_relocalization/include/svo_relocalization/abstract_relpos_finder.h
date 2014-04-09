

#ifndef SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF
#define SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF

#include <memory>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

#include <svo_relocalization/frame.h>

namespace reloc
{

class AbstractRelposFinder
{
public:
  AbstractRelposFinder () {};
  virtual ~AbstractRelposFinder () {};

  virtual void removeFrame(int frame_id) = 0;

  virtual void addFrame(const FrameSharedPtr &frame) = 0;

  virtual Sophus::SE3 findRelpos(
      FrameSharedPtr frame_query,
      const FrameSharedPtr& frame_best_match) = 0;

private:

};

typedef std::shared_ptr<AbstractRelposFinder> AbstractRelposFinderSharedPtr;

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF */

