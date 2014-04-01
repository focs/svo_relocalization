
#include <svo_relocalization/CC_place_finder.h>

namespace reloc
{

CCPlaceFinder::CCPlaceFinder ()
{

}

CCPlaceFinder::~CCPlaceFinder()
{

}

void CCPlaceFinder::addFrame(const std::vector<cv::Mat>& img_pyr, const Sophus::SE3& T_frame_wordl, int id)
{
  // Not sure which level of the pyramid should be used, taking first
  cv::Size s (40, 30);
  int idx = 0;
  while (idx < img_pyr.size() &&
      img_pyr.at(idx).size().width > s.width)
    ++idx;

  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = convertToSmallBlurryImage(img_pyr.at(0));

  // Enqueue new image
  ImagePoseId tmp_ip = {im_small_blur_0mean, T_frame_wordl, id};
  images_.push_back(tmp_ip);
}

cv::Mat CCPlaceFinder::getSmallBlurryImage(int idx)
{

  if (idx < images_.size())
  {
    return images_.at(idx).image;
  } 
  else 
  {
    //Need to do something else maybe?
    return cv::Mat();
  }

}

CCPlaceFinder::ImagePoseId& CCPlaceFinder::findBestMatch(const cv::Mat& queryImage)
{
  std::vector<ImagePoseId>::iterator image_it;
  std::vector<ImagePoseId>::iterator best_pair = images_.begin();

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

cv::Mat CCPlaceFinder::convertToSmallBlurryImage(const cv::Mat& img)
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
