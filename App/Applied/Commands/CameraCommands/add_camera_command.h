#ifndef ADD_CAMERA_COMMAND_H
#define ADD_CAMERA_COMMAND_H

#include "camera_command.h"

#include <memory>

class AddCameraCommand : public CameraCommand {
public:
  AddCameraCommand() = default;

  AddCameraCommand(const Vec3f& center, const Vec3f& eye, const Vec3f& up) {
    _camera = std::make_shared<Camera>(center, eye, up);
    //    qDebug() << "AddCameraCommand created";
  }

  AddCameraCommand(std::shared_ptr<BaseObject> camera) : _camera(camera) {}

  ~AddCameraCommand() = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    //    qDebug() << "AddCameraCommand::execute()";
    mainManager->addObject(_camera);
  }

private:
  std::shared_ptr<BaseObject> _camera;
};

#endif  // ADD_CAMERA_COMMAND_H
