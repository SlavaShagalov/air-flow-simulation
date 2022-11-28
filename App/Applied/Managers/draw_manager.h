#ifndef DRAW_MANAGER_H
#define DRAW_MANAGER_H

#include "base_manager.h"

#include <Domains/Applied/Drawer/drawer.h>
#include <Domains/Applied/SceneObjects/camera.h>
#include <Domains/Applied/Scene/scene.h>

#include <Domains/Applied/global_types.h>

class DrawManager : public BaseManager {
public:
  DrawManager() = default;
  DrawManager(const DrawManager&) = delete;
  DrawManager(DrawManager&&) = delete;

  ~DrawManager() = default;

  DrawManager& operator=(const DrawManager&) = delete;
  DrawManager& operator=(DrawManager&&) = delete;

  void drawScene(const std::shared_ptr<Scene>& scene,
                 const std::shared_ptr<BaseDrawer> drawer,
                 const std::shared_ptr<Camera> camera, const DrawMode mode,
                 const ParticleMode particleMode);
};

#endif  // DRAW_MANAGER_H
