
#ifndef SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL
#define SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL

#include <svo_relocalization/abstract_relocalizer.h>

#include <svo_relocalization/frame.h>
#include <svo_relocalization/abstract_place_finder.h>
#include <svo_relocalization/abstract_relpos_finder.h>


namespace reloc
{

class MultipleRelocalizer
{

public:
  MultipleRelocalizer(
      AbstractPlaceFinderSharedPtr place_finder,
      AbstractRelposFinderSharedPtr relpos_finder);

  virtual ~MultipleRelocalizer ();

  void removeFrame(int frame_id);
  
  void addFrame(FrameSharedPtr frame);

  /// Method to relocalize applying a place finder and a relative pose finder
  /// frame_query does not need to have all attributes initialized, depending
  /// the used methods some of them will not be used.

  // Not sure what is it returning....
  bool relocalize(
      const FrameSharedPtr &frame_query,
      int &id_out);

private:
  AbstractPlaceFinderSharedPtr place_finder_;
  AbstractRelposFinderSharedPtr relpos_finder_;
  
};

}

#endif /* end of include guard: SVO_RELOCALIZATION_MULTIPLE_RELOCALIZER_H_WHVCZAKL */
