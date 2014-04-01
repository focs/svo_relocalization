
#ifndef SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2
#define SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2

#include <svo_relocalization/abstract_relpos_finder.h>

#include <vikit/abstract_camera.h>
#include <sophus/se2.h>

namespace reloc
{

class ESMRelposFinder : public AbstractRelposFinder
{
public:
  ESMRelposFinder(vk::AbstractCamera *camera_model);
  virtual ~ESMRelposFinder();

  virtual Sophus::SE3 findRelpos (
      cv::Mat query_img,
      cv::Mat template_img
      );

  static Sophus::SE3 findSE3(Sophus::SE2 t, vk::AbstractCamera *camera_model);

private:
  vk::AbstractCamera *camera_model_;
};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_ESM_RELPOS_FINDER_H_H4JI3OS2 */

