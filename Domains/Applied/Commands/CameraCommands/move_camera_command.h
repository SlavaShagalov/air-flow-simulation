#ifndef MOVE_CAMERA_COMMAND_H
#define MOVE_CAMERA_COMMAND_H

#include "camera_command.h"

#include <cstddef>

class MoveCameraCommand : public CameraCommand {
public:
  MoveCameraCommand() = delete;

  MoveCameraCommand(std::shared_ptr<BaseObject> camera, double shift_x,
                    double shift_y)
      : _camera(camera), shift_x(shift_x), shift_y(shift_y) {}

  ~MoveCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->moveObject(_camera, shift_x, shift_y, 0);
  }

private:
  std::shared_ptr<BaseObject> _camera;
  double shift_x, shift_y;
};

#endif  // MOVE_CAMERA_COMMAND_H
