
#include <svo_relocalization/multiple_relocalizer.h>
#include <cstdio>


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
  palace_finder_->removeFrame(frame_id);
  relpos_finder_->removeFrame(frame_id);
}

void MultipleRelocalizer::addFrame(FrameSharedPtr frame)
{
  palace_finder_->addFrame(frame);
  relpos_finder_->addFrame(frame);
}

bool MultipleRelocalizer::relocalize(
    const FrameSharedPtr &frame_query,
    int &id_out)
{
  FrameSharedPtr found_frame;
  found_frame = place_finder_->findPlace(frame_query);

  relpos_finder_->findRelpos(frame_query, found_frame);
}

} /* reloc */ 
