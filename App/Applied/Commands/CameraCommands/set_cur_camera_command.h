#ifndef SET_CUR_CAMERA_COMMAND_H
#define SET_CUR_CAMERA_COMMAND_H

#include "camera_command.h"

#include <cstddef>

class SetCurCameraCommand : public CameraCommand {
public:
  SetCurCameraCommand() = delete;

  explicit SetCurCameraCommand(size_t cameraId) : _cameraId(cameraId) {}

  ~SetCurCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->setCurCamera(_cameraId);
  }

private:
  size_t _cameraId;
};

#endif  // SET_CUR_CAMERA_COMMAND_H
