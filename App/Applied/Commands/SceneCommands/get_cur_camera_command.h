#ifndef GET_CUR_CAMERA_COMMAND_H
#define GET_CUR_CAMERA_COMMAND_H

#include "scene_command.h"

#include <cstddef>

class GetCurCameraCommand : public SceneCommand {
public:
  explicit GetCurCameraCommand(std::shared_ptr<Camera>& camera)
      : _camera(camera) {}

  ~GetCurCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    _camera = mainManager->getCurCamera();
  }

private:
  std::shared_ptr<Camera>& _camera;
};
#endif  // GET_CUR_CAMERA_COMMAND_H
