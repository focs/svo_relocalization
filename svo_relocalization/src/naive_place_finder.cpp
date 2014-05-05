
#include <svo_relocalization/naive_place_finder.h>

namespace reloc
{
NaivePlaceFinder::NaivePlaceFinder ()
{

}

NaivePlaceFinder::~NaivePlaceFinder ()
{

}

void NaivePlaceFinder::removeFrame(int frame_id)
{

}

// Need to add points
void NaivePlaceFinder::addFrame(const FrameSharedPtr &frame_data)
{
  frames.push_back(frame_data);
}

/// Should return the id of the most similar frame
FrameSharedPtr NaivePlaceFinder::findPlace(FrameSharedPtr frame_query)
{
  double min_dist = std::numeric_limits<float>::max();
  int frame_idx = 0; // Hope that there is one frame at least

  for (size_t i = 0; i < frames.size(); ++i)
  {
    double dist = (frame_query->T_frame_world_.translation() - frames.at(i)->T_frame_world_.translation()).norm();
    if (dist < min_dist) 
    {
      min_dist = dist;
      frame_idx = i;
    }
  }

  return frames.at(frame_idx);
}

} /* reloc */ 
