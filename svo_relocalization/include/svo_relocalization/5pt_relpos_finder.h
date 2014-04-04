

#ifndef SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT
#define SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT

#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class FivePtRelposFinder : public AbstractRelposFinder
{
public:
  FivePtRelposFinder ();
  virtual ~FivePtRelposFinder ();

  virtual Sophus::SE3 findRelpos (
      cv::Mat query_img,
      cv::Mat template_img
      );

private:

};

} /* reloc */ 


#endif /* end of include guard: SVO_RELOCALIZER_5PT_RELPOS_FINDER_CPP_PL4PDRUT */
