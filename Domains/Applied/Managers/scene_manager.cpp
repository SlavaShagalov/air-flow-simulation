#include "scene_manager.h"

#include <Domains/Applied/Exceptions/load_exceptions.h>

SceneManager::SceneManager() : _scene(std::make_shared<Scene>()) {}

std::shared_ptr<Camera> SceneManager::getCurCamera() const {
  auto camera_ptr = _curCamera.lock();

  if (!camera_ptr)
    throw NoCameraError(__FILE__, __LINE__, "Current camera doesn't added");

  return camera_ptr;
}

std::shared_ptr<Scene> SceneManager::getScene() const { return this->_scene; }

void SceneManager::setScene(std::shared_ptr<Scene> new_scene) {
  _scene = std::move(new_scene);
}

void SceneManager::setCurCamera(size_t cameraId) {
  auto it = _scene->begin();
  std::advance(it, cameraId);
  _curCamera = std::dynamic_pointer_cast<Camera>(*it);
}

std::shared_ptr<BaseObject> SceneManager::getObjectById(size_t id) {
  auto obj_iter = _scene->begin();
  std::advance(obj_iter, id);
  std::shared_ptr<BaseObject> obj = *obj_iter;

  return obj;
}

void SceneManager::removeObject(size_t id) {
  auto it = _scene->begin();
  std::advance(it, id);
  _scene->removeObject(it);
}
