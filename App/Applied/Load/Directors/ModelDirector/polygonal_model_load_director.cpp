#include "polygonal_model_load_director.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <QDebug>

#include <Domains/Applied/Load/Loaders/ModelLoader/polygonal_model_file_loader.h>

#include <Domains/Applied/SceneObjects/Model/polygonal_model.h>

#include <Domains/Applied/Load/Builders/Model/file_polygon_model_builder.h>

PolygonalModelLoadDirector::PolygonalModelLoadDirector() {
  _builder = std::make_shared<FilePolygonalModelBuilder>();
  _loader = std::make_shared<PolygonalModelObjFileLoader>();
}

void loadTextureImage() {}

std::shared_ptr<BaseObject>
PolygonalModelLoadDirector::load(const std::string& fileName) {
  std::shared_ptr<PolygonalModel> model = std::make_shared<PolygonalModel>();
  auto& components = model->components();

  std::ifstream in;
  in.open(fileName, std::ifstream::in);
  if (in.fail()) {
    qDebug() << "Fail";
    return nullptr;
  }

  std::string line;
  while (!in.eof()) {
    std::getline(in, line);
    std::istringstream iss(line.c_str());
    char trash;
    if (!line.compare(0, 2, "v ")) {
      iss >> trash;
      Vec3f v;
      iss >> v.x();
      iss >> v.y();
      iss >> v.z();
      components->addVertex(v);
    } else if (!line.compare(0, 3, "vn ")) {
      iss >> trash >> trash;
      Vec3f n;
      for (int i = 0; i < 3; i++)
        iss >> n[i];
      components->addNormal(n);
    } else if (!line.compare(0, 3, "vt ")) {
      iss >> trash >> trash;
      //      Point2D_D t;
      //      iss >> t.x();
      //      iss >> t.y();
      //      components->addTexture(t);
    } else if (!line.compare(0, 2, "f ")) {
      Face f;
      int itrash, idx, ndx;
      iss >> trash;
      while (iss >> idx >> trash >> itrash >> trash >> ndx) {
        idx--;  // in wavefront obj all indices start at 1, not zero
        ndx--;
        f.push_back(idx, ndx);
      }
      components->addFace(f);
    }
  }

  loadTextureImage();

  return model;
}

void PolygonalModelLoadDirector::loadVertices() {
  Vec3f vertex;
  size_t count = _loader->loadCount();
  for (size_t i = 0; i < count; i++) {
    vertex = _loader->loadVertex();
    _builder->buildVertex(vertex);
  }
}

void PolygonalModelLoadDirector::loadFaces() {
  Edge edge;
  size_t count = _loader->loadCount();
  for (size_t i = 0; i < count; i++) {
    edge = _loader->loadEdge();
    _builder->buildEdge(edge);
  }
}

void PolygonalModelLoadDirector::loadCenter() {
  Vec3f center = _loader->loadVertex();
  _builder->buildVertex(center);
}
