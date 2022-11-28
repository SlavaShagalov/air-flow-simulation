#ifndef FILE_CAMERA_BUILDER_H
#define FILE_CAMERA_BUILDER_H

#include "camera_builder.h"

#include <memory>

#include <Domains/Applied/SceneObjects/camera.h>

class FileCameraBuilder : public CameraBuilder {
public:
  FileCameraBuilder() = default;
  ~FileCameraBuilder() = default;

  void reset();

  std::shared_ptr<BaseObject> get();

  void buildPosition(Vec3f);

  virtual void buildVertex(Vec3f) {}

  virtual void buildEdge(Edge) {}

  virtual void buildCenter(Vec3f) {}

private:
  std::shared_ptr<Camera> _camera;
};

#endif  // FILE_CAMERA_BUILDER_H
