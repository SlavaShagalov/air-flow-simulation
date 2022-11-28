#include "draw_manager.h"

#include <utility>
#include <memory>

#include <App/Applied/Primitives/Vector4D/vector_4d.hpp>
#include <App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>

#include <App/Applied/Visitors/draw_visitor.h>

void DrawManager::drawScene(const std::shared_ptr<Scene>& scene,
                            const std::shared_ptr<BaseDrawer> drawer,
                            const std::shared_ptr<Camera> camera,
                            const DrawMode mode,
                            const ParticleMode particleMode) {
  //  qDebug() << "DrawManager::drawScene() START";
  // get lights
  const auto lights = scene->lights();

  drawer->clear();

  // draw objects
  auto visitor = DrawVisitor(drawer, camera, lights, mode, particleMode);
  scene->objects()->accept(visitor);

  drawer->update();

  //  qDebug() << "DrawManager::drawScene() END";
}
