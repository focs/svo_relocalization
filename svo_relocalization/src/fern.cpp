
#include <svo_relocalization/fern.h>

#include <random>
#include <chrono>

namespace reloc
{
  
Fern::Fern (int num_tests, int max_x, int max_y) :
  max_x_(max_x),
  max_y_(max_y)
{

  if (num_tests > 20)
  {
    std::cerr << "Number of tests too large" << std::endl;
    exit(-1);
  }

  operants1.resize(num_tests);
  operants2.resize(num_tests);


  for (size_t i = 0; i < num_tests; ++i)
  {

    operants1[i] = cv::Point(
        rand() % max_x_ - max_x_/2,
        rand() % max_y_ - max_y_/2);

    operants2[i] = cv::Point(
        rand() % max_x_ - max_x_/2,
        rand() % max_y_ - max_y_/2);
  }
}

Fern::~Fern ()
{

}

uint32_t Fern::evaluetePatch (const cv::Mat &patch)
{

  cv::Point center (patch.cols/2, patch.rows/2);
  uint32_t fern = 0;
  if (patch.cols >= max_x_ && patch.rows >= max_y_)
  {
    for (size_t i = 0; i < operants1.size(); ++i)
    {
      //Shift bits
      fern <<= 1;
      if (patch.at<uint8_t>(operants1.at(i)+center)
          < patch.at<uint8_t>(operants2.at(i)+center))
      {
        //Change last bit
        fern++;
      }

    }
  }
  else
  {
    //wrong patch size
    std::cerr << "Patch too small" << std::endl;
  }

  return fern;
}

} /* reloc */ 
