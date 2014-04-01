

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

  virtual Sophus::SE3 findRelpos (
      cv::Mat template_img,
      cv::Mat queryi_img) = 0;

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_ABSTRACT_RELPOS_FINDER_H_RZXA0WTF */

