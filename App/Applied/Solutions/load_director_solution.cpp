#include "load_director_solution.h"

#include <fstream>
#include <QDebug>

#include <Domains/Applied/Load/Directors/CameraDirector/camera_load_director.h>

#include <Domains/Applied/Load/Directors/ModelDirector/polygonal_model_load_director.h>
#include <Domains/Applied/Load/Directors/ModelDirector/wireframe_model_load_director.h>

#include <Domains/Applied/Exceptions/load_exceptions.h>

#define CAMERA_DIRECTOR_ID 1
#define WIREFRAME_MODEL_DIRECTOR_ID 2
#define POLYGONAL_MODEL_DIRECTOR_ID 3

template <typename Tprod>
bool LoadDirectorSolution::registration(size_t id) {
  return callbacks.emplace(id, std::make_shared<Tprod>()).second;
}

void LoadDirectorSolution::initId() {
  registration<CameraLoadDirector>(CAMERA_DIRECTOR_ID);
  registration<WireframeModelLoadDirector>(WIREFRAME_MODEL_DIRECTOR_ID);
  registration<PolygonalModelLoadDirector>(POLYGONAL_MODEL_DIRECTOR_ID);
}

std::shared_ptr<BaseLoadDirector>
LoadDirectorSolution::createDirector(size_t id) {
  CallBackMap::const_iterator it = callbacks.find(id);

  if (it == callbacks.end())
    throw ConfigError(__FILE__, __LINE__, "Invalid Id");

  return it->second;
}
