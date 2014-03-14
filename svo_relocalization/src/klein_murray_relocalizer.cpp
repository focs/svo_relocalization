
#include <svo_relocalization/klein_murray_relocalizter.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <sophus/se2.h>

#include <svo_relocalization/img_aling_se2.h>

namespace reloc
{
 

KMRelocalizer::KMRelocalizer ()
{

}

KMRelocalizer::~KMRelocalizer()
{

}

void KMRelocalizer::addFrame(const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id)
{
  // Not sure which level of the pyramid should be used, taking first
  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = convertToSmallBluryImage(img_pyr.at(0));

  // Enqueue new image
  ImagePoseId tmp_ip = {im_small_blur_0mean, T_frame_wordl, id};
  images_.push_back(tmp_ip);

}

bool KMRelocalizer::relocalize(
    const std::vector<cv::Mat>& img_pyr,
    const Sophus::SE3& T_frame_world_estimate,
    Sophus::SE3& T_frame_wordl_out,
    int& id_out)
{

  // Compare current image to all stored images
  // Taking first element of the pyramid for now
  cv::Mat query_img = convertToSmallBluryImage(img_pyr.at(0));
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

  // convert se2 to se3 with xy translation and rotation over z axes
  Eigen::Vector3d se2_values = Sophus::SE2::vee(T_template_query.matrix());
  Eigen::Matrix< double, 6, 1 > se3_values;
  // Set X translation
  se3_values[0] = se2_values[0];
  // Set Y translation
  se3_values[1] = se2_values[1];
  // Z translation and X, Y rotations are set to 0
  se3_values[2] = 0;
  se3_values[3] = 0;
  se3_values[4] = 0;
  // Set Z rotation
  se3_values[5] = se2_values[2];

  // Apply found rotation to the known frame rotation
  T_frame_wordl_out = Sophus::SE3::exp(se3_values).inverse() * best_match.T_f_w;
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

cv::Mat KMRelocalizer::convertToSmallBluryImage(const cv::Mat& img)
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

