
#include <svo_relocalization/klein_murray_relocalizter.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <sophus/se2.h>

#include <svo_relocalization/img_aling_se2.h>

KMRelocalizer::KMRelocalizer ()
{

}

KMRelocalizer::~KMRelocalizer()
{

}

void KMRelocalizer::addFrame(const cv::Mat& img, const Sophus::SE3& pose, int id)
{
  // Downsample image
  cv::Mat im_small;
  cv::resize(img, im_small, cv::Size(40,30));

  // Blur image
  cv::Mat im_small_blur;
  cv::GaussianBlur(im_small, im_small_blur, cv::Size(3,3), 2.5, 2.5);

  // Substract mean to make it 0 mean
  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = im_small_blur - cv::mean(im_small_blur);

  // Enqueue new image
  ImagePose tmp_ip (im_small_blur_0mean, pose);
  images_.push_back(tmp_ip);

}

bool KMRelocalizer::relocalize(
    const cv::Mat& img,
    const Sophus::SE3& last_pose,
    Sophus::SE3& pose_out,
    int& id_out)
{

  // Compare current image to all stored images
  ImagePose best_match = findBestMatch(img);
  
  // Find best transformation between current frame and best match
  // Model is from img to best_match
  SecondOrderMinimizationSE2 rotation_minimizer(img, best_match.first);
  // Model set to 0
  Sophus::SE2 rotation_model;
  rotation_minimizer.optimize(rotation_model);

  Eigen::Vector3d se2_values = Sophus::SE2::vee(rotation_model.matrix());
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
  pose_out = Sophus::SE3::exp(se3_values).inverse() * best_match.second;

}

KMRelocalizer::ImagePose& KMRelocalizer::findBestMatch(const cv::Mat& queryImage)
{
  std::list<ImagePose>::iterator image_it;
  std::list<ImagePose>::iterator best_pair = images_.begin();

  float best_diff = std::numeric_limits<float>::max();
  // Iterate over the images an compare with the query image
  for (image_it = images_.begin(); image_it != images_.end(); image_it++)
  {
    cv::Mat diff_im = queryImage - image_it->first;
    float diff = cv::sum(diff_im.mul(diff_im))[0];

    if (diff < best_diff)
    {
      best_diff = diff;
      best_pair = best_pair;
    }
  }
}
