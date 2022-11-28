#include "file_polygon_model_builder.h"

#include <Domains/Applied/SceneObjects/Model/wireframe_model.h>

FilePolygonalModelBuilder::FilePolygonalModelBuilder() {}

void FilePolygonalModelBuilder::reset() {
  _modelComponents = std::make_shared<WireframeModelComponents>();
}

std::shared_ptr<BaseObject> FilePolygonalModelBuilder::get() {
  std::shared_ptr<BaseObject> model =
  std::make_shared<WireframeModel>(_modelComponents, _center);
  return model;
}

void FilePolygonalModelBuilder::buildVertex(Vec3f vertex) {
  _modelComponents->addVertex(vertex);
}

void FilePolygonalModelBuilder::buildEdge(Edge e) {
  _modelComponents->addEdge(e);
}

void FilePolygonalModelBuilder::buildCenter(Vec3f center) {
  _center = center;
}
