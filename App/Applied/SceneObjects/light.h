#ifndef LIGHT_H
#define LIGHT_H

#include "invisible_object.h"

#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>

class Light : public InvisibleObject {
private:
  Vec3f _dir;

public:
  Light() : _dir(0, 0, 0) {}
  Light(const Light& other) = default;
  Light(Light&& other) = default;

  // getters
  virtual Vec3f& center() override { return _center; }
  virtual Vec3f center() const override { return _center; }
  Vec3f dir() const { return _dir; }
  Vec3f& dir() { return _dir; }

  virtual void accept(BaseTransformVisitor& visitor) override {}
  virtual void accept(BaseDrawVisitor& visitor) override {}

  Light(const Vec3f& dir) : _dir(dir) {}
};

#endif  // LIGHT_H
