
#ifndef SVO_RELOCALIZATION_ABSTRACT_RELOCALIZER_H_RA9MYHW8
#define SVO_RELOCALIZATION_ABSTRACT_RELOCALIZER_H_RA9MYHW8

#include <opencv2/opencv.hpp>
#include <sophus/se3.h> 

#include <svo_relocalization/frame.h>

namespace reloc
{

/// Abstract Class (interface) to implement a relocalization method
class AbstractRelocalizer
{
public:
  AbstractRelocalizer (){};
  virtual ~AbstractRelocalizer (){};

  virtual void removeFrame(int frame_id) = 0;
  
  virtual void addFrame(FrameSharedPtr frame) = 0;

  virtual void train () {};

  virtual bool relocalize(
      FrameSharedPtr frame_query,
      Sophus::SE3 &pose_out,
      int &id_out) = 0;
};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZATION_ABSTRACT_RELOCALIZATOR_H_RA9MYHW8 */

