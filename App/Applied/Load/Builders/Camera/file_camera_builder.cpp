#include "file_camera_builder.h"

void FileCameraBuilder::reset() { this->_camera = std::make_shared<Camera>(); }

std::shared_ptr<BaseObject> FileCameraBuilder::get() { return _camera; }

void FileCameraBuilder::buildPosition(Vec3f pos) {
  //  _camera->setPosition(pos);
}
