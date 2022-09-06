#ifndef INVISIBLE_OBJECT_H
#define INVISIBLE_OBJECT_H

#include "base_object.h"

class InvisibleObject : public BaseObject {
public:
  InvisibleObject() = default;
  InvisibleObject(const Vec3f& center) : BaseObject(center) {}
  virtual ~InvisibleObject() override = default;

  bool isVisible() const override { return false; }
};

#endif  // INVISIBLE_OBJECT_H
