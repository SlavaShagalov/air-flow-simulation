#pragma once

#include <App/Applied/Color/color.h>
#include <App/Applied/Drawer/drawer.h>
#include <App/Applied/SceneObjects/Model/polygonal_model.h>
#include <App/Applied/SceneObjects/Model/wireframe_model.h>
#include <App/Applied/SceneObjects/ParticleSystem/particle_system.h>
#include <App/Applied/SceneObjects/camera.h>
#include <App/Applied/SceneObjects/composite.h>
#include <App/Applied/SceneObjects/light.h>
#include <App/Applied/global_types.h>

#include <App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>
#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <App/Applied/Primitives/Vector4D/vector_4d.hpp>
#include <QDebug>
#include <memory>

#include "base_draw_visitor.h"

class DrawVisitor : public BaseDrawVisitor {
 public:
  DrawVisitor(const std::shared_ptr<BaseDrawer>& drawer, const std::shared_ptr<Camera>& camera,
              const std::vector<std::shared_ptr<Light>>& lights, const DrawMode mode, const ParticleMode particleMode,
              const int y_min, const int y_max)
      : _drawer(drawer),
        _camera(camera),
        _lights(lights),
        _mode(mode),
        _particleMode(particleMode),
        _ymin(y_min),
        _ymax(y_max) {
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

  int _ymin;
  int _ymax;
};
