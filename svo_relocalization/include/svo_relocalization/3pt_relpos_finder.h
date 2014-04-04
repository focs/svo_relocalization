
#ifndef SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ
#define SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ


#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class ThreePtRelposFinder : public AbstractRelposFinder
{
public:

  struct Options {
    size_t n_iterations;
    double reprojection_error;
    Options()
    : n_iterations(300),
      reprojection_error(2.0)
    {} 
  } options_;

  ThreePtRelposFinder ();
  virtual ~ThreePtRelposFinder ();

  virtual Sophus::SE3 findRelpos (
      cv::Mat query_img,
      cv::Mat template_img);

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ */

