

#ifndef SVO_RELOCALIZER_YAML_PARSE_H_AIARECHO
#define SVO_RELOCALIZER_YAML_PARSE_H_AIARECHO

#include <list>
#include <svo/frame.h>
#include <svo/point.h>
#include <vikit/abstract_camera.h>

void readYaml (std::string path, std::list<svo::FramePtr> &frames, vk::AbstractCamera *cam);
void writeYaml (std::string path, const std::list<svo::FramePtr> &frames);

#endif /* end of include guard: SVO_RELOCALIZER_YAML_PARSE_H_AIARECHO */

