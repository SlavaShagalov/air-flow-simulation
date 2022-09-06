#ifndef DRAW_VISITOR_H
#define DRAW_VISITOR_H

#include <QDebug>

#include <memory>

#include "base_draw_visitor.h"

#include <Domains/Applied/SceneObjects/camera.h>
#include <Domains/Applied/SceneObjects/composite.h>
#include <Domains/Applied/SceneObjects/light.h>
#include <Domains/Applied/SceneObjects/Model/polygonal_model.h>
#include <Domains/Applied/SceneObjects/Model/wireframe_model.h>
#include <Domains/Applied/SceneObjects/ParticleSystem/particle_system.h>

#include <Domains/Applied/Drawer/drawer.h>
#include <Domains/Applied/Color/color.h>

#include <Domains/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>
#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <Domains/Applied/Primitives/Vector4D/vector_4d.hpp>

#include <Domains/Applied/global_types.h>

class DrawVisitor : public BaseDrawVisitor {
public:
  DrawVisitor(const std::shared_ptr<BaseDrawer>& drawer,
              const std::shared_ptr<Camera>& camera,
              const std::vector<std::shared_ptr<Light>>& lights,
              const DrawMode mode, const ParticleMode particleMode)
      : _drawer(drawer)
      , _camera(camera)
      , _lights(lights)
      , _mode(mode)
      , _particleMode(particleMode) {
    // TODO: here need to init matrices
  }

  virtual void visit(const WireframeModel& model) override;
  virtual void visit(const PolygonalModel& model) override;
  virtual void visit(const ParticleSystem& ps) override;

  void wireframeDraw(const PolygonalModel& model);
  void simpleDraw(const PolygonalModel& model);
  void gourandDraw(const PolygonalModel& model);

private:
  std::shared_ptr<BaseDrawer> _drawer;
  std::shared_ptr<Camera> _camera;
  std::vector<std::shared_ptr<Light>> _lights;
  DrawMode _mode;
  ParticleMode _particleMode;
};

#endif  // DRAW_VISITOR_H