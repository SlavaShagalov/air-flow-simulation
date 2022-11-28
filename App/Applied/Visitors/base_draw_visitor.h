#ifndef BASE_DRAW_VISITOR_H
#define BASE_DRAW_VISITOR_H

#include <QDebug>

#include <memory>

class WireframeModel;
class PolygonalModel;
class ParticleSystem;

class BaseDrawVisitor {
public:
  virtual void visit(const WireframeModel& model) = 0;
  virtual void visit(const PolygonalModel& model) = 0;
  virtual void visit(const ParticleSystem& ps) = 0;
};

#endif  // BASE_DRAW_VISITOR_H
