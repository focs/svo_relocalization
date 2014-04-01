

#ifndef SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR
#define SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR

#include <svo_relocalization/abstract_place_finder.h>

#include <vector>

#include <vikit/abstract_camera.h>

namespace reloc
{

class CCPlaceFinder : public AbstractPlaceFinder
{
public:
  CCPlaceFinder ();
  virtual ~CCPlaceFinder ();

  // Need to add points
  // T_frame_world not used
  virtual void addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_world, int id);

  cv::Mat getSmallBlurryImage(int idx);

private:
  /// Structure used to save data in a std::list
  struct ImagePoseId
  {
    cv::Mat image;
    Sophus::SE3 T_f_w;
    int id;
  };

  /// Find best match with small blurred images
  ImagePoseId& findBestMatch(const cv::Mat& queryImage);
  /// Convert to "small blurry image"
  cv::Mat convertToSmallBlurryImage(const cv::Mat& img);

  std::vector<ImagePoseId> images_; //<! List of images included so far with its pose and id

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_CC_PLACE_FINDER_H_CFMT6DRR */



