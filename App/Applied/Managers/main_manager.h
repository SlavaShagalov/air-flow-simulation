#ifndef MAIN_MANAGER_H
#define MAIN_MANAGER_H

#include <stddef.h>
#include <memory>
#include <QDebug>

#include "base_manager.h"
#include "draw_manager.h"
#include "load_manager.h"
#include "scene_manager.h"
#include "transform_manager.h"
#include "manager_creator.h"

#include <App/Applied/global_types.h>

class MainManager : public BaseManager {
private:
  std::shared_ptr<LoadManager> _loadManager;
  std::shared_ptr<DrawManager> _drawManager;
  std::shared_ptr<TransformManager> _transformManager;
  std::shared_ptr<SceneManager> _sceneManager;

public:
  static std::shared_ptr<MainManager> instance() {
    return std::make_shared<MainManager>(MainManager());
  }
  ~MainManager() = default;

  // base object actions
  void moveObject(const std::shared_ptr<BaseObject>& obj, const double& dx,
                  const double& dy, const double& dz) {
    _transformManager->moveObject(obj, dx, dy, dz);
  }
  void scaleObject(const std::shared_ptr<BaseObject>& obj, const double& kx,
                   const double& ky, const double& kz) {
    _transformManager->scaleObject(obj, kx, ky, kz);
  }
  void rotateObject(const std::shared_ptr<BaseObject>& obj, const double& ox,
                    const double& oy, const double& oz) {
    _transformManager->rotateObject(obj, ox, oy, oz);
  }
  void transformObject(const std::shared_ptr<BaseObject>& obj,
                       const Mtr4x4f& mtr) {
    _transformManager->transformObject(obj, mtr);
  }

  std::shared_ptr<BaseObject> loadObject(const std::string& fileName,
                                         const size_t diractorId) {
    //    qDebug() << "MainManager::loadObject()";
    return _loadManager->load(fileName, diractorId);
  }
  void addObject(const std::shared_ptr<BaseObject>& object) {
    _sceneManager->addObject(object);
  }
  void addLight(const std::shared_ptr<Light> light) {
    _sceneManager->addLight(light);
  }
  void removeObject(size_t id) { _sceneManager->removeObject(id); }

  // model astions
  //  size_t countModels() const { return _sceneManager->getModelsCount(); }

  // camera actions
  //  size_t countCameras() const { return _sceneManager->getCamerasCount(); }
  void setCurCamera(size_t id) { _sceneManager->setCurCamera(id); }
  std::shared_ptr<Camera> getCurCamera() const {
    return _sceneManager->getCurCamera();
  }

  // scene actions
  std::shared_ptr<BaseObject> getSceneObject(size_t id) {
    return _sceneManager->getObjectById(id);
  }
  std::shared_ptr<Scene> getScene() { return _sceneManager->getScene(); }

  // draw actions
  void drawScene(const std::shared_ptr<Scene>& scene,
                 const std::shared_ptr<BaseDrawer> drawer,
                 const std::shared_ptr<Camera> camera, const DrawMode mode,
                 const ParticleMode particleMode) {
    _drawManager->drawScene(scene, drawer, camera, mode, particleMode);
  }

protected:
  MainManager();
  //  MainManager(MainManager&) = delete;
  //  MainManager(const MainManager&) = delete;
};

#endif  // MAIN_MANAGER_H
