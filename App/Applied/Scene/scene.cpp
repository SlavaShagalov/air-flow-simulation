#include "scene.h"

#include <QDebug>

#include <utility>

#include <Domains/Applied/SceneObjects/base_object.h>

void Scene::addObject(const std::shared_ptr<BaseObject>& object) {
  //  qDebug() << "Scene::addObject()";

  _objects->add(object);
}

void Scene::removeObject(const Iterator& it) { _objects->remove(it); }
