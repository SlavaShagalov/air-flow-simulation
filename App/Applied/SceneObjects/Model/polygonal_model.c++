#include "polygonal_model.h"

PolygonalModel::PolygonalModel(
std::shared_ptr<PolygonalModelComponents> components, const Vec3f& center)
    : _components(std::move(components)) {
  _center = center;
}

const std::shared_ptr<PolygonalModelComponents>
PolygonalModel::components() const {
  return _components;
}
