#ifndef LOAD_CAMERA_COMMAND_H
#define LOAD_CAMERA_COMMAND_H

#include "camera_command.h"

#include <string>

class LoadCameraCommand : public CameraCommand {
public:
  LoadCameraCommand() = delete;

  LoadCameraCommand(std::shared_ptr<BaseObject> camera,
                    const std::string& filename)
      : _camera(camera), _filename(filename) {
    this->_directorId = 1;
  }

  ~LoadCameraCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    _camera = mainManager->loadObject(_filename, _directorId);
  }

private:
  std::shared_ptr<BaseObject> _camera;
  std::string _filename;
  size_t _directorId;
};

#endif  // LOAD_CAMERA_COMMAND_H
