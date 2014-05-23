
#include <svo_relocalization/esm_relpos_finder.h>
#include <svo_relocalization/img_aling_se2.h>
#include <svo_relocalization/se2_align_se3.h>

#include <opencv2/core/eigen.hpp>
namespace reloc
{

ESMRelposFinder::ESMRelposFinder(vk::AbstractCamera *camera_model) :
  camera_model_(camera_model)
{

}

ESMRelposFinder::~ESMRelposFinder()
{

}
  
void ESMRelposFinder::removeFrame(int frame_id)
{
}

void ESMRelposFinder::addFrame(const FrameSharedPtr &frame)
{
}

cv::Mat warpOS3 (const cv::Mat &im, const Sophus::SO3 &t, vk::AbstractCamera *cam)
{
  cv::Mat im_out (im.rows, im.cols, im.type());

  for (size_t v = 0; v < im.rows; ++v)
  {
    for (size_t u = 0; u < im.cols; ++u)
    {
      Eigen::Vector2d p_out (u,v);
      Eigen::Vector2d p_in = cam->world2cam(t.inverse() * cam->cam2world(p_out));

      int u_in, v_in;
      u_in = static_cast<int>(p_in[0]);
      v_in = static_cast<int>(p_in[1]);

      if (u_in < im.cols && u_in >=0 &&
          v_in < im.rows && v_in >=0)
      {
        im_out.at<uint8_t>(v,u) = im.at<uint8_t>(v_in, u_in);
      }
      else
      {
        im_out.at<uint8_t>(v,u) = 0;
      }
    }
  }

  return im_out;

}


Sophus::SE3 ESMRelposFinder::findRelpos(
    FrameSharedPtr frame_query,
    const FrameSharedPtr& frame_best_match)
{

  // Model set to 0
  Sophus::SE2 se2_T_query_template;

  for (size_t i = frame_query->img_pyr_.size()-1; i >= options_.pyr_lvl_; --i)
  {
    se2_T_query_template.translation() = se2_T_query_template.translation() * 2;
    // Optimizing from first parameter towards second
    SecondOrderMinimizationSE2 motion_estimator(
        frame_best_match->img_pyr_.at(i),
        frame_query->img_pyr_.at(i));
    motion_estimator.optimizeGaussNewton(se2_T_query_template);
    //std::cout << "Found SE2: " << se2_T_query_template << std::endl;
  } 

  se2_T_query_template.translation() =
    se2_T_query_template.translation()
    * (1 << options_.pyr_lvl_);

  // Apply found rotation to the known frame rotation
  Sophus::SE3 se3_T_query_template;
  se3_T_query_template = findSE3(
      se2_T_query_template,
      camera_model_,
      options_.n_iter_se2_to_se3_);


  /****************************************************************************/
  cv::Mat im_, im2;
  frame_best_match->img_pyr_.at(0).convertTo(im_, CV_8U);
  frame_query->img_pyr_.at(0).convertTo(im2, CV_8U);

  Sophus::SE2 center_translation (0, Eigen::Vector2d(-im_.cols/2, -im_.rows/2));
  Sophus::SE2 temp_model = center_translation.inverse()
    * se2_T_query_template
    * center_translation;
 
  // Warp template image
  cv::Mat warp_mat(2, 3, CV_32F);
  cv::eigen2cv(
      Eigen::Matrix<double,2,3>(temp_model.matrix().topRows(2)),
      warp_mat);

  cv::Mat im_warp;
  //cv::warpAffine(im_, im_warp, warp_mat, im_warp.size());
  cv::warpAffine(
      im_,
      im_warp,
      warp_mat,
      im_warp.size(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      0);

  cv::Mat im_show;
  cv::Mat im_se2_warp;
  cv::Mat im_so3_warp;
  im_warp.convertTo(im_se2_warp, CV_8UC1);
  im_so3_warp = warpOS3(im_, se3_T_query_template.so3() , camera_model_);

  //im_show.push_back(im2);
  im_show.push_back(im_se2_warp);
  im_show.push_back(im_so3_warp);
  //cv::imshow("so3 aligment", warpOS3(im_, se3_T_query_template.so3() , camera_model_));
  //im_show.push_back(im_warp);
  //

  //std::vector<cv::Mat> channels;
  //cv::Mat color;

  //std::cout << "color:   " << color.size().width << " x " << color.size().height << " x " << color.channels() << " " << color.type() << std::endl;
  cv::Mat z = cv::Mat::zeros(im_.size(), CV_8UC1);

  //std::cout << "Red:   " << z.size().width << " x " << z.size().height << " x " << z.channels() <<" " << color.type() <<  std::endl;
  //std::cout << "Green: " << im_.size().width << " x " << im_.size().height << " x " << im_.channels() << " " << color.type() << std::endl;
  //std::cout << "Blue:  " << im_se2_warp.size().width << " x " << im_se2_warp.size().height << " x " << im_se2_warp.channels() << " " << color.type() << std::endl;

  //channels.push_back(z);

  //cv::merge(channels, color);

  cv::Mat img, g, fin_img;
    img = cv::imread("/home/fox/Pictures/error_dist.png",CV_LOAD_IMAGE_GRAYSCALE);
    std::vector<cv::Mat> channels;

    g = cv::Mat::zeros(cv::Size(img.cols, img.rows), CV_8UC1);

  channels.push_back(z);
  channels.push_back(im_se2_warp);
  channels.push_back(im2);

  merge(channels,  fin_img);
  imshow("hola", fin_img);
  cv::imshow("aliniation", im_show);
  cv::waitKey(1);
  
  /****************************************************************************/

  return se3_T_query_template.inverse();
  //return se3_T_query_template * frame_best_match->T_frame_world_;
}

Sophus::SE3 ESMRelposFinder::findSE3(
    Sophus::SE2 t,
    vk::AbstractCamera *camera_model,
    uint32_t n_iter)
{
  std::cout << "Starting SE2 to SE3" << std::endl;

  Sophus::SE3 model;
  SE2toSE3 s23 (t, camera_model);
  s23.n_iter_ = n_iter;
  s23.optimizeGaussNewton(model);


  return model;
}

} /* reloc */ 
