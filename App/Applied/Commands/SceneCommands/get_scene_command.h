#ifndef GET_SCENE_COMMAND_H
#define GET_SCENE_COMMAND_H

#include "scene_command.h"

#include <cstddef>

class GetSceneCommand : public SceneCommand {
public:
  explicit GetSceneCommand(std::shared_ptr<Scene>& scene) : _scene(scene) {}

  ~GetSceneCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    _scene = mainManager->getScene();
  }

private:
  std::shared_ptr<Scene>& _scene;
};
#endif  // GET_SCENE_COMMAND_H
