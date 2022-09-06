#ifndef POLYGONAL_MODEL_H
#define POLYGONAL_MODEL_H

#include "model.h"
#include "polygonal_model_components.h"

#include <Domains/Applied/Visitors/base_draw_visitor.h>
#include <Domains/Applied/Visitors/base_transform_visitor.h>

class PolygonalModel : public Model {
public:
  PolygonalModel()
      : _components(std::make_shared<PolygonalModelComponents>()) {}
  PolygonalModel(const PolygonalModel& other) = default;
  PolygonalModel(PolygonalModel&& other) = default;

  PolygonalModel(std::shared_ptr<PolygonalModelComponents> components,
                 const Vec3f& center);

  ~PolygonalModel() = default;

  virtual void accept(BaseDrawVisitor& visitor) override {
    visitor.visit(*this);
  }
  virtual void accept(BaseTransformVisitor& visitor) override {
    visitor.visit(*this);
  }

  const std::shared_ptr<PolygonalModelComponents> components() const;

private:
  friend class DrawVisitor;
  friend class TransformVisitor;

  std::shared_ptr<PolygonalModelComponents> _components;
};

#endif  // POLYGONAL_MODEL_H
