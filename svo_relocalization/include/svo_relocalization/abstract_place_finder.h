
#ifndef SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO
#define SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

namespace reloc
{

class AbstractPlaceFinder
{
public:
  AbstractPlaceFinder () {};
  virtual ~AbstractPlaceFinder () {};

private:
  
  // Need to add points
  virtual void addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_world, int id) = 0;
  /// Should return the id of the most similar frame
  int findPlace();
};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO */

