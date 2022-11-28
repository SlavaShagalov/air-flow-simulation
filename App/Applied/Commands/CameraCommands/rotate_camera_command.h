#ifndef ROTATE_CAMERA_COMMAND_H
#define ROTATE_CAMERA_COMMAND_H

#include "camera_command.h"

#include <cstddef>

class RotateCameraCommand : public CameraCommand {
public:
  RotateCameraCommand() = delete;

  RotateCameraCommand(std::shared_ptr<BaseObject> camera, float ax, float ay,
                      float az)
      : _camera(camera), _ax(ax), _ay(ay), _az(az) {}

  ~RotateCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->rotateObject(_camera, _ax, _ay, _az);
  }

private:
  std::shared_ptr<BaseObject> _camera;
  float _ax, _ay, _az;
};

#endif  // ROTATE_CAMERA_COMMAND_H
