

#ifndef SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR
#define SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR

#include <svo_relocalization/abstract_place_finder.h>

#include <vector>

namespace reloc
{

class CCPlaceFinder : public AbstractPlaceFinder
{
public:
  CCPlaceFinder ();
  virtual ~CCPlaceFinder ();

  void removeFrame(int frame_id);
  void addFrame(const FrameSharedPtr &frame);

  FrameSharedPtr findPlace(FrameSharedPtr frame_query);
  cv::Mat getSmallBlurryImage(uint32_t idx);

private:

  /// Structure used to save data in a std::list
  struct ExtendedFrame
  {
    cv::Mat smallBlurryImage;
    FrameSharedPtr data;
  };

  /// Find best match with small blurred images
  void findBestMatch(const cv::Mat& queryImage, ExtendedFrame &result);

  /// Convert to "small blurry image"
  cv::Mat convertToSmallBlurryImage(const std::vector<cv::Mat> &img_pyr);

  std::vector<ExtendedFrame> images_; //<! List of images included so far with its pose and id

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR */



