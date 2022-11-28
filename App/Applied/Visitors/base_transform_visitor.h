#ifndef BASE_TRANSFORM_VISITOR_H
#define BASE_TRANSFORM_VISITOR_H

#include <QDebug>

#include <memory>

class WireframeModel;
class PolygonalModel;
class Camera;
class Composite;

class BaseTransformVisitor {
public:
  virtual void visit(Camera& camera) = 0;
  virtual void visit(WireframeModel& model) = 0;
  virtual void visit(PolygonalModel& model) = 0;
  virtual void visit(Composite& composite) = 0;
};

#endif  // BASE_TRANSFORM_VISITOR_H
