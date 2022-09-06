#ifndef TRANSFORM_VISITOR_H
#define TRANSFORM_VISITOR_H

#include <memory>

#include "base_transform_visitor.h"

#include <Domains/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>

#include <Domains/Applied/SceneObjects/Model/polygonal_model.h>
#include <Domains/Applied/SceneObjects/Model/wireframe_model.h>
#include <Domains/Applied/SceneObjects/camera.h>
#include <Domains/Applied/SceneObjects/composite.h>

class TransformVisitor : public BaseTransformVisitor {
public:
  TransformVisitor() = default;
  TransformVisitor(const Mtr4x4f& mtr) {
    _mtr = std::make_shared<Mtr4x4f>(mtr);
  }

  ~TransformVisitor() = default;

  virtual void visit(Camera& camera) override;
  virtual void visit(WireframeModel& model) override;
  virtual void visit(PolygonalModel& model) override;
  virtual void visit(Composite& composite) override {}

private:
  std::shared_ptr<Mtr4x4f> _mtr;
};

#endif  // TRANSFORM_VISITOR_H
