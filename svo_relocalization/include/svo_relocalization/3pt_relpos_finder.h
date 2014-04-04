
#ifndef SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ
#define SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ


#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class TrheePtRelposFinder : public AbstractRelposFinder
{
public:
  TrheePtRelposFinder ();
  virtual ~TrheePtRelposFinder ();

  virtual Sophus::SE3 findRelpos (
      cv::Mat query_img,
      cv::Mat template_img
      );

private:

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_3PT_RELPOS_FINDER_H_VFMEXCUJ */

