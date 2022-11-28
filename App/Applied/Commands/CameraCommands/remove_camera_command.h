#ifndef REMOVE_CAMERA_COMMAND_H
#define REMOVE_CAMERA_COMMAND_H

#include "camera_command.h"

#include <cstddef>

class RemoveCameraCommand : public CameraCommand {
public:
  RemoveCameraCommand() = delete;

  explicit RemoveCameraCommand(std::size_t cameraId) : _cameraId(cameraId) {}

  ~RemoveCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->removeObject(_cameraId);
  }

private:
  std::size_t _cameraId;
};

#endif  // REMOVE_CAMERA_COMMAND_H
