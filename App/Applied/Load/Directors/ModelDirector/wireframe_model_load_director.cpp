#include "wireframe_model_load_director.h"

#include <QDebug>

#include <App/Applied/Load/Builders/Model/file_wireframe_model_builder.h>
#include <App/Applied/Load/Loaders/ModelLoader/wireframe_model_file_loader.h>

WireframeModelLoadDirector::WireframeModelLoadDirector() {
  _builder = std::make_shared<FileWireframeModelBuilder>();
  _loader = std::make_shared<WireframeModelFileLoader>();
}

std::shared_ptr<BaseObject>
WireframeModelLoadDirector::load(const std::string& fileName) {
  _builder->reset();
  _loader->openFile(fileName);

  loadVertices();
  //  qDebug() << "Success load vertices";
  loadCenter();
  //  qDebug() << "Success load center";
  loadEdges();
  //  qDebug() << "Success load edges";

  _loader->closeFile();

  auto model = _builder->get();

  return model;
}

void WireframeModelLoadDirector::loadVertices() {
  Vec3f vertex;
  size_t count = _loader->loadCount();
  for (size_t i = 0; i < count; i++) {
    vertex = _loader->loadVertex();
    _builder->buildVertex(vertex);
  }
}

void WireframeModelLoadDirector::loadEdges() {
  Edge edge;
  size_t count = _loader->loadCount();
  for (size_t i = 0; i < count; i++) {
    edge = _loader->loadEdge();
    _builder->buildEdge(edge);
  }
}

void WireframeModelLoadDirector::loadCenter() {
  Vec3f center = _loader->loadVertex();
  _builder->buildVertex(center);
}
