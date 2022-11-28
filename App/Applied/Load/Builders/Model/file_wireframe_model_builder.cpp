#include "file_wireframe_model_builder.h"

#include <App/Applied/SceneObjects/Model/wireframe_model.h>

FileWireframeModelBuilder::FileWireframeModelBuilder() {}

void FileWireframeModelBuilder::reset() {
  _modelComponents = std::make_shared<WireframeModelComponents>();
}

std::shared_ptr<BaseObject> FileWireframeModelBuilder::get() {
  std::shared_ptr<BaseObject> model =
  std::make_shared<WireframeModel>(_modelComponents, _center);
  return model;
}

void FileWireframeModelBuilder::buildVertex(Vec3f vertex) {
  _modelComponents->addVertex(vertex);
}

void FileWireframeModelBuilder::buildEdge(Edge e) {
  _modelComponents->addEdge(e);
}

void FileWireframeModelBuilder::buildCenter(Vec3f center) {
  _center = center;
}
