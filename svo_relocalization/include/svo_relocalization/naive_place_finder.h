#ifndef SVO_RELOCALIZATION_NAIVE_PLACE_FINDER_H_ZMUEWJHD
#define SVO_RELOCALIZATION_NAIVE_PLACE_FINDER_H_ZMUEWJHD

#include <svo_relocalization/abstract_place_finder.h>

#include <vector>

#include <svo_relocalization/frame.h>

namespace reloc
{
class NaivePlaceFinder : public AbstractPlaceFinder
{
public:
  NaivePlaceFinder ();
  virtual ~NaivePlaceFinder ();

  void removeFrame(int frame_id);
  
  // Need to add points
  void addFrame(const FrameSharedPtr &frame_data);
  
  /// Should return the id of the most similar frame
  FrameSharedPtr findPlace(FrameSharedPtr frame_query);

private:
  std::vector<FrameSharedPtr> frames; 
};

} /* reloc */ 
#endif /* end of include guard: SVO_RELOCALIZATION_NAIVE_PLACE_FINDER_H_ZMUEWJHD */

