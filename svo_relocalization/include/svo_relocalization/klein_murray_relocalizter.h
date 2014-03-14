
#ifndef SVO_RELOCALIZATION_KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5
#define SVO_RELOCALIZATION_KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5

#include <list>

#include <svo_relocalization/abstract_relocalizer.h>

class TestConMatKMRelocalizer;
namespace reloc
{

/// Implementation of a relocalizer using the technique from Klein and Murray
class KMRelocalizer : AbstractRelocalizer
{

public:
  KMRelocalizer ();
  virtual ~KMRelocalizer ();

  void addFrame (const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id);

  bool relocalize(
      const std::vector<cv::Mat>& query_img_pyr,
      const Sophus::SE3& T_frame_world_estimate,
      Sophus::SE3& T_frame_wordl_out,
     int& id_out);

  // Friend test class (allow access private atributes, methods and datastructures)
  friend class ::TestConMatKMRelocalizer;

private:

  /// Structure used to save data in a std::list
  struct ImagePoseId
  {
    cv::Mat image;
    Sophus::SE3 T_f_w;
    int id;
  };

  /// Find best match with small blured images
  ImagePoseId& findBestMatch(const cv::Mat& queryImage);
  /// Convert to "small blury image"
  cv::Mat convertToSmallBluryImage(const cv::Mat& img);

  std::list<ImagePoseId> images_; //<! List of images included so far with its pose and id
};

  
} /* reloc */ 
#endif /* end of include guard: SVO_RELOCALIZATION_KLEIN_MURRAY_RELOCALIZER_H_LC4ZIRV5 */

