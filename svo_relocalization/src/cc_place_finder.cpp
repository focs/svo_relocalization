
#include <svo_relocalization/cc_place_finder.h>

namespace reloc
{

CCPlaceFinder::CCPlaceFinder ()
{

}

CCPlaceFinder::~CCPlaceFinder()
{

}

void CCPlaceFinder::removeFrame(int frame_id)
{
  //TO BE DONE!!!
}

uint32_t findInPyr(const std::vector<cv::Mat> &img_pyr, cv::Size s)
{
  uint32_t idx = 0;
  while (idx+1 < img_pyr.size() &&
      img_pyr.at(idx).size().width > s.width)
    ++idx;

  return idx;
}

void CCPlaceFinder::addFrame(const FrameSharedPtr &frame)
{

  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = convertToSmallBlurryImage(frame->img_pyr_);

  // Enqueue new image
  ExtendedFrame tmp_ip = {im_small_blur_0mean, frame};
  images_.push_back(tmp_ip);
}

FrameSharedPtr CCPlaceFinder::findPlace(FrameSharedPtr frame_query)
{

  cv::Mat im_query_small_blur_0mean;
  im_query_small_blur_0mean = convertToSmallBlurryImage(frame_query->img_pyr_);

  ExtendedFrame best_match;
  best_match = findBestMatch(im_query_small_blur_0mean);

  return best_match.data;

}

cv::Mat CCPlaceFinder::getSmallBlurryImage(uint32_t idx)
{

  if (idx < images_.size())
  {
    return images_.at(idx).smallBlurryImage;
  } 
  else 
  {
    //Need to do something else maybe?
    return cv::Mat();
  }

}

CCPlaceFinder::ExtendedFrame& CCPlaceFinder::findBestMatch(const cv::Mat& queryImage)
{
  std::vector<ExtendedFrame>::iterator image_it;
  std::vector<ExtendedFrame>::iterator best_pair = images_.begin();

  float best_diff = std::numeric_limits<float>::max();
  // Iterate over the images an compare with the query image
  for (image_it = images_.begin(); image_it != images_.end(); image_it++)
  {
    cv::Mat diff_im = queryImage - image_it->smallBlurryImage;
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

cv::Mat CCPlaceFinder::convertToSmallBlurryImage(const std::vector<cv::Mat>& img_pyr)
{
  // Search level of the pyramid that is similar to the used one
  cv::Size s (40, 30);
  uint32_t idx;
  idx = findInPyr(img_pyr, s);

  cv::Mat im_float;
  img_pyr.at(idx).convertTo(im_float, CV_32F);

  cv::Mat im_small;
  if (im_float.size().height > s.height || im_float.size().width > s.width)
  {
    cv::resize(im_float, im_small, s);
  }
  else
  {
    im_small = im_float;
  }


  // Blur image
  cv::Mat im_small_blur;
  cv::GaussianBlur(im_small, im_small_blur, cv::Size(3,3), 2.5, 2.5);

  // Substract mean to make it 0 mean
  cv::Mat im_small_blur_0mean;
  im_small_blur_0mean = im_small_blur - cv::mean(im_small_blur);
  
  return im_small_blur_0mean;
}      

} /* reloc */ 
