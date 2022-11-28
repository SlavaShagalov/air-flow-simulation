#ifndef CAMERA_BUILDER_H
#define CAMERA_BUILDER_H

#include <App/Applied/Load/Builders/object_builder.h>

class CameraBuilder : public ObjectBuilder {
public:
  CameraBuilder() = default;

  virtual ~CameraBuilder() = default;

  virtual void reset() = 0;

  virtual std::shared_ptr<BaseObject> get() = 0;

  virtual void buildPosition(Vec3f) = 0;
};
#endif  // CAMERA_BUILDER_H
