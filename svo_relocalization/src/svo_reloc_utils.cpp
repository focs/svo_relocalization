
#include <svo_relocalization/svo_reloc_utils.h>

#include <svo/feature.h>
#include <svo/point.h>

void svo2reloc (svo::FramePtr svo_frame, reloc::FrameSharedPtr reloc_frame)
{

  reloc_frame->img_pyr_ = svo_frame->img_pyr_;
  reloc_frame->id_ = svo_frame->id_;
  reloc_frame->T_frame_world_ = svo_frame->T_f_w_;

  for (svo::Features::iterator it = svo_frame->fts_.begin(); it != svo_frame->fts_.end(); it++)
  {
    reloc::Feature f;
    if ((*it)->point != NULL)
    {

      f.point_id_ = (*it)->point->id_;
      f.point_w_ = (*it)->point->pos_;
    }

    f.px_ = (*it)->px;
    f.pyr_lvl_ = (*it)->level;

    reloc_frame->features_.push_back(f);
    
  }

}
