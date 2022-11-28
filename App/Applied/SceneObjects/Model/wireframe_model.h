#ifndef WIREFRAME_MODEL_H
#define WIREFRAME_MODEL_H

#include "model.h"
#include "wireframe_model_components.h"

#include <memory>

#include <Domains/Applied/Visitors/base_draw_visitor.h>
#include <Domains/Applied/Visitors/base_transform_visitor.h>

#include <Domains/Applied/Color/color.h>

class DrawVisitor;

class WireframeModel : public Model {
public:
  WireframeModel();

  WireframeModel(std::shared_ptr<WireframeModelComponents> components,
                 const Vec3f& center, const Color& color = Color());
  WireframeModel(const WireframeModel& model);

  ~WireframeModel() = default;

  virtual void accept(BaseDrawVisitor& visitor) override {
    visitor.visit(*this);
  }
  virtual void accept(BaseTransformVisitor& visitor) override {
    visitor.visit(*this);
  }

  const std::shared_ptr<WireframeModelComponents> getComponents() const;

private:
  friend class DrawVisitor;
  friend class TransformVisitor;

  std::shared_ptr<WireframeModelComponents> _components;
  Color _color;
};

#endif  // WIREFRAME_MODEL_H
