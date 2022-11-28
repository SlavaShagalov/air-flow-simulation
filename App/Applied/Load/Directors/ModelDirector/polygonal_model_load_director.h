#ifndef POLYGONAL_MODEL_LOAD_DIRECTOR_H
#define POLYGONAL_MODEL_LOAD_DIRECTOR_H

#include "model_load_director.h"

#include <App/Applied/SceneObjects/Model/polygonal_model.h>

class PolygonalModelLoadDirector : public ModelLoadDirector {
public:
  PolygonalModelLoadDirector();
  ~PolygonalModelLoadDirector() = default;

  std::shared_ptr<BaseObject> load(const std::string& src_name) override;

private:
  void loadVertices();
  void loadFaces();
  void loadCenter();
};

#endif  // POLYGONAL_MODEL_LOAD_DIRECTOR_H
