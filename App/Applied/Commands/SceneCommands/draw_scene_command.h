#ifndef DRAW_SCENE_COMMAND_H
#define DRAW_SCENE_COMMAND_H

#include "scene_command.h"

#include <Domains/Applied/global_types.h>

class DrawSceneCommand : public SceneCommand {
public:
  DrawSceneCommand() = delete;

  explicit DrawSceneCommand(std::shared_ptr<Scene>& scene,
                            std::shared_ptr<BaseDrawer> drawer,
                            std::shared_ptr<Camera> curCamera,
                            const DrawMode mode,
                            const ParticleMode particleMode)
      : _drawer(drawer)
      , _scene(scene)
      , _curCamera(curCamera)
      , _mode(mode)
      , _particleMode(particleMode) {}

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->drawScene(_scene, _drawer, _curCamera, _mode, _particleMode);
  }

  ~DrawSceneCommand() override = default;

private:
  std::shared_ptr<BaseDrawer> _drawer;
  std::shared_ptr<Scene>& _scene;
  std::shared_ptr<Camera> _curCamera;
  DrawMode _mode;
  ParticleMode _particleMode;
};

#endif  // DRAW_SCENE_COMMAND_H
