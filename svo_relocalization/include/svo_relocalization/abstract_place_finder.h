
#ifndef SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO
#define SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO

#include <memory>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

#include <svo_relocalization/frame.h>

namespace reloc
{


class AbstractPlaceFinder
{
public:
  AbstractPlaceFinder () {};
  virtual ~AbstractPlaceFinder () {};

  virtual void removeFrame(int frame_id) = 0;
  
  // Need to add points
  virtual void addFrame(const FrameSharedPtr &frame_data) = 0;
  
  /// Should return the id of the most similar frame
  virtual FrameSharedPtr findPlace(FrameSharedPtr frame_query) = 0;

};

typedef std::shared_ptr<AbstractPlaceFinder> AbstractPlaceFinderSharedPtr;

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO */

