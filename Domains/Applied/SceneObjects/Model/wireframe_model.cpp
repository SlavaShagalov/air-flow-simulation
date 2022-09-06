#include "wireframe_model.h"

WireframeModel::WireframeModel()
    : _components(std::make_shared<WireframeModelComponents>())
    , _color(Color()) {}

WireframeModel::WireframeModel(
std::shared_ptr<WireframeModelComponents> components, const Vec3f& center,
const Color& color)
    : _components(std::move(components)), _color(color) {
  _center = center;
}

WireframeModel::WireframeModel(const WireframeModel& other)
    : _components(other._components), _color(Color()) {}

const std::shared_ptr<WireframeModelComponents>
WireframeModel::getComponents() const {
  return _components;
}
