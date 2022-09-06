#ifndef CAMERA_H
#define CAMERA_H

#include "invisible_object.h"

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>

#include <Domains/Applied/Visitors/base_draw_visitor.h>
#include <Domains/Applied/Visitors/base_transform_visitor.h>

class Camera : public InvisibleObject {
public:
  Camera() = default;
  ~Camera() = default;

  Camera(const Vec3f& center, const Vec3f& eye, const Vec3f& up)
      : InvisibleObject(center), _eye(eye), _up(up) {}

  virtual void accept(BaseTransformVisitor& visitor) override {
    visitor.visit(*this);
  }
  virtual void accept(BaseDrawVisitor& visitor) override {}

  virtual Vec3f& center() override { return _center; }
  virtual Vec3f center() const override { return _center; }
  Vec3f& eye() { return _eye; }
  Vec3f eye() const { return _eye; }
  Vec3f& up() { return _up; }
  Vec3f up() const { return _up; }

  void setEye(const Vec3f& eye) { _eye = eye; }
  void setUp(const Vec3f& up) { _up = up; }
  void setCenter(const Vec3f& center) { _center = center; }

private:
  Vec3f _eye = { 0, 0, 0 };
  Vec3f _up = { 0, 1, 0 };

  friend class TransformVisitor;
};

#endif  // CAMERA_H
