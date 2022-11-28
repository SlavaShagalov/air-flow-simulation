#ifndef SCENE_H
#define SCENE_H

#include <App/Applied/Containers/matrix/iterator/matrix_iterator.hpp>
#include <App/Applied/SceneObjects/base_object.h>
#include <App/Applied/SceneObjects/composite.h>
#include <App/Applied/SceneObjects/light.h>
#include <memory>
#include <vector>

class Scene {
  using Iterator = BaseObject::Iterator;
  using ConstIterator = BaseObject::ConstIterator;

private:
  std::shared_ptr<Composite> _objects = std::make_shared<Composite>();

  std::vector<std::shared_ptr<Light>> _lights;

public:
  Scene() = default;

  ~Scene() = default;

  void addObject(const std::shared_ptr<BaseObject>& object);
  void addLight(const std::shared_ptr<Light> light) {
    _lights.push_back(light);
  }

  void removeObject(const Iterator& it);

  Iterator begin() { return _objects->begin(); }
  Iterator end() { return _objects->end(); }

  std::shared_ptr<Composite> objects() { return _objects; }
  std::vector<std::shared_ptr<Light>>& lights() { return _lights; }
};

#endif  // SCENE_H
