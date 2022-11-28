#ifndef MODEL_H
#define MODEL_H

#include <Domains/Applied/SceneObjects/visible_object.h>

class Model : public VisibleObject {
public:
  Model() = default;
  Model(const Model& other) = default;
  Model(Model&& other) = default;

  virtual ~Model() = default;

  Vec3f center() const override { return _center; }
  Vec3f& center() override { return _center; }
};

#endif  // MODEL_H
