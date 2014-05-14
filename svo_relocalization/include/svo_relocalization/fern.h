#ifndef SVO_RELOCALIZER_FERN_H_PRVK2HPG
#define SVO_RELOCALIZER_FERN_H_PRVK2HPG

#include <vector>
#include <opencv2/opencv.hpp>

namespace reloc
{

class Fern
{
public:
  Fern (int num_tests, int max_x, int max_y);
  virtual ~Fern ();

  uint32_t evaluetePatch (const cv::Mat &patch);

  int getMaxX () { return max_x_; };
  int getMaxY () { return max_y_; };
  int getNumTests() { return operants1.size(); };

private:
  // First operant positions
  std::vector <cv::Point2i> operants1;
  // Second operant positions
  std::vector <cv::Point2i> operants2;

  int max_x_;
  int max_y_;

};

} /* reloc */ 

#endif /* end of include guard: SVO_RELOCALIZER_FERN_H_PRVK2HPG */

