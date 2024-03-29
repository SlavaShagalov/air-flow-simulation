#include "camera_load_director.h"

#include <App/Applied/Load/Builders/Camera/file_camera_builder.h>

#include <App/Applied/Load/Loaders/CameraLoader/camera_file_loader.h>

CameraLoadDirector::CameraLoadDirector() {
  _builder = std::make_shared<FileCameraBuilder>();
  _loader = std::make_shared<CameraFileLoader>();
}

std::shared_ptr<BaseObject>
CameraLoadDirector::load(const std::string& srcFileName) {
  _builder->reset();
  _loader->openFile(srcFileName);

  Vec3f pos = _loader->loadVertex();
  _builder->buildPosition(pos);

  _loader->closeFile();

  auto camera = _builder->get();

  return camera;
}
