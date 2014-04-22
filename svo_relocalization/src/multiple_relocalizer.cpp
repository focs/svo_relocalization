
#include <cstdio>
#include <svo_relocalization/multiple_relocalizer.h>

namespace reloc
{
  
MultipleRelocalizer::MultipleRelocalizer (
    AbstractPlaceFinderSharedPtr place_finder,
    AbstractRelposFinderSharedPtr relpos_finder) :
  place_finder_(place_finder),
  relpos_finder_(relpos_finder)
{

}

MultipleRelocalizer::~MultipleRelocalizer ()
{
}

void MultipleRelocalizer::removeFrame (int frame_id)
{
  place_finder_->removeFrame(frame_id);
  relpos_finder_->removeFrame(frame_id);
}

void MultipleRelocalizer::addFrame(FrameSharedPtr frame)
{
  place_finder_->addFrame(frame);
  relpos_finder_->addFrame(frame);
}

bool MultipleRelocalizer::relocalize(
    FrameSharedPtr frame_query,
    int &id_out)
{
  FrameSharedPtr found_frame;
  found_frame = place_finder_->findPlace(frame_query);

  id_out = found_frame->id_;
  std::cout << "transformation of the found frame: " << std::endl << found_frame->T_frame_world_;

  // This will push the found SE3 into frame_query
  frame_query->T_frame_world_ = relpos_finder_->findRelpos(frame_query, found_frame);

  // We dont know if the result is correct or no for now
  return true;
}

} /* reloc */ 