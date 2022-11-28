#ifndef CAMERA_LOAD_DIRECTOR_H
#define CAMERA_LOAD_DIRECTOR_H

#include <App/Applied/SceneObjects/base_object.h>

#include <App/Applied/Load/Directors/base_load_director.h>

class CameraLoadDirector : public BaseLoadDirector {
public:
  CameraLoadDirector();
  ~CameraLoadDirector() = default;

  std::shared_ptr<BaseObject> load(const std::string& src_name) override;
};

#endif  // CAMERA_LOAD_DIRECTOR_H
