

#ifndef SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF
#define SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

namespace reloc
{

class AbstractRelposFinder
{
public:
  AbstractRelposFinder () {};
  virtual ~AbstractRelposFinder () {};

  virtual void removeFrame(int frame_id) {};

  virtual void addFrame(FrameDataPtr frame_data) = 0;

  virtual Sophus::SE3 findRelpos(
      const FrameDataPtr& frame_query,
      const FrameDataPtr& frame_best_match,
      const Sophus::SE3& T_frame_world_estimate) = 0;

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF */

