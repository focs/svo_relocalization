
#include <svo_relocalization/klein_murray_relocalizter.h>

#include <Eigen/Core>

#include <svo_relocalization/img_aling_se2.h>
#include <svo_relocalization/se2_align_se3.h>

namespace reloc
{
 

KMRelocalizer::KMRelocalizer (vk::AbstractCamera *camera_model) :
  camera_model_(camera_model)
{

}

KMRelocalizer::~KMRelocalizer()
{

}

void KMRelocalizer::addFrame(const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id)
{
  // Not sure which level of the pyramid should be used, taking first
  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = convertToSmallBlurryImage(img_pyr.at(0));

  // Enqueue new image
  ImagePoseId tmp_ip = {im_small_blur_0mean, T_frame_wordl, id};
  images_.push_back(tmp_ip);
}

Sophus::SE3 KMRelocalizer::findSE3(Sophus::SE2 t, vk::AbstractCamera *camera_model)
{
  Sophus::SE3 model;
  SE2toSE3 s23 (t, camera_model);
  s23.n_iter_ = 3;
  s23.optimizeGaussNewton(model);

  return model;
}

bool KMRelocalizer::relocalize(
    const std::vector<cv::Mat>& img_pyr,
    const Sophus::SE3& T_frame_world_estimate,
    Sophus::SE3& T_frame_wordl_out,
    int& id_out)
{
  // Compare current image to all stored images
  // Taking first element of the pyramid for now
  cv::Mat query_img = convertToSmallBlurryImage(img_pyr.at(0));
  ImagePoseId best_match = findBestMatch(query_img);
  
  // Find best transformation between current frame and best match
  // Model is from img to best_match
  //std::cout << "query image: " << query_img << std::endl;
  //std::cout << "best match image: " << best_match.image << std::endl;
  SecondOrderMinimizationSE2 motion_estimator(query_img, best_match.image);
  // Model set to 0
  Sophus::SE2 T_template_query;
  //std::cout << "Template transformation: " << T_template_query << std::endl;
  motion_estimator.optimize(T_template_query);

  // Apply found rotation to the known frame rotation
  T_frame_wordl_out = findSE3(T_template_query, camera_model_).inverse() * best_match.T_f_w;
  id_out = best_match.id;

  // For now
  return true;
}

KMRelocalizer::ImagePoseId& KMRelocalizer::findBestMatch(const cv::Mat& queryImage)
{
  std::list<ImagePoseId>::iterator image_it;
  std::list<ImagePoseId>::iterator best_pair = images_.begin();

  float best_diff = std::numeric_limits<float>::max();
  // Iterate over the images an compare with the query image
  for (image_it = images_.begin(); image_it != images_.end(); image_it++)
  {
    cv::Mat diff_im = queryImage - image_it->image;
    // Sum of squared differences 
    float diff = cv::sum(diff_im.mul(diff_im))[0];

    if (diff < best_diff)
    {
      //std::cout << "best diff: " << diff << std::endl;
      best_diff = diff;
      best_pair = image_it;
    }
  }

  //std::cout << "best match id: " << best_pair->id << std::endl;

  return *best_pair;
}

cv::Mat KMRelocalizer::convertToSmallBlurryImage(const cv::Mat& img)
{
  cv::Mat im_float;
  img.convertTo(im_float, CV_32F);
  // Downsample image
  cv::Mat im_small;
  cv::resize(im_float, im_small, cv::Size(40,30));

  // Blur image
  cv::Mat im_small_blur;
  cv::GaussianBlur(im_small, im_small_blur, cv::Size(3,3), 2.5, 2.5);

  // Substract mean to make it 0 mean
  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = im_small_blur - cv::mean(im_small_blur);
  
  return im_small_blur_0mean;
}      

} /* reloc */ 

