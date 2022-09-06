#include "composite.h"

#include <QDebug>

Composite::Composite(const std::shared_ptr<BaseObject>& component) {
  _objects.push_back(component);
}

Composite::Composite(const std::vector<std::shared_ptr<BaseObject>>& vector) {
  _objects = vector;
}

bool Composite::add(const std::shared_ptr<BaseObject>& component) {
  //  qDebug() << "Composite::add()";
  _objects.push_back(component);
  return true;
}

bool Composite::remove(const Iterator& iterator) {
  _objects.erase(iterator);
  return true;
}

void Composite::updateCenter() {
  _center = Vec3f(0, 0, 0);
  size_t count = 0;

  for (const auto& element : _objects) {
    _center = _center + element->center();
    count++;
  }

  _center =
  Vec3f(_center.x() / count, _center.y() / count, _center.z() / count);
}
