#include "main_manager.h"

// protected
MainManager::MainManager() {
  _loadManager = ManagerCreator<LoadManager>().getManager();
  _drawManager = ManagerCreator<DrawManager>().getManager();
  _transformManager = ManagerCreator<TransformManager>().getManager();
  _sceneManager = ManagerCreator<SceneManager>().getManager();
}
