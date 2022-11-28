#ifndef COUNT_CAMERAS_COMMAND_H
#define COUNT_CAMERAS_COMMAND_H

#include "camera_command.h"

#include <memory>
#include <cstddef>

class CountCamerasCommand : public CameraCommand {
public:
  CountCamerasCommand() = delete;

  explicit CountCamerasCommand(size_t& count) : _count(count) {}

  ~CountCamerasCommand() = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    //    _count = mainManager->countCameras();
  }

private:
  size_t& _count;
};

#endif  // COUNT_CAMERAS_COMMAND_H
