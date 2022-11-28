#ifndef SCENE_MANAGER_H
#define SCENE_MANAGER_H

#include "base_manager.h"

#include <memory>

#include <QDebug>

#include <Domains/Applied/SceneObjects/base_object.h>
#include <Domains/Applied/SceneObjects/camera.h>

#include <Domains/Applied/Scene/scene.h>

using Iterator = std::vector<std::shared_ptr<BaseObject>>::iterator;

class SceneManager : public BaseManager {
public:
  SceneManager();
  SceneManager(const SceneManager&) = delete;
  SceneManager& operator=(const SceneManager&) = delete;
  ~SceneManager() override = default;

  std::shared_ptr<Scene> getScene() const;
  std::shared_ptr<Camera> getCurCamera() const;
  std::shared_ptr<BaseObject> getObjectById(size_t id);

  void setScene(std::shared_ptr<Scene> scene);
  void setCurCamera(size_t camera_id);

  void addObject(const std::shared_ptr<BaseObject> object) {
    _scene->addObject(object);
  }

  void addLight(const std::shared_ptr<Light> light) { _scene->addLight(light); }

  void removeObject(size_t id);

private:
  std::shared_ptr<Scene> _scene;
  std::weak_ptr<Camera> _curCamera;
};

#endif  // SCENE_MANAGER_H
