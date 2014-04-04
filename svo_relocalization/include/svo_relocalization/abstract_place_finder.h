
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
  
  virtual void removeFrame(int frame_id) = 0;
  
  // Need to add points
  virtual void addFrame(FrameDataPtr frame_data) = 0;
  
  /// Should return the id of the most similar frame
  virtual FrameDataPtr findPlace(
      const std::vector<cv::Mat>& query_img_pyr,
      const Sophus::SE3& T_frame_world_estimate);

};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_PLACE_FINDER_H_Q5PUSNLO */

