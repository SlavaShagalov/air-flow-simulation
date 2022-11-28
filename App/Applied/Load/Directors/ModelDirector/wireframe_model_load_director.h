#ifndef WIREFRAME_MODEL_LOAD_DIRECTOR_H
#define WIREFRAME_MODEL_LOAD_DIRECTOR_H

#include "model_load_director.h"

class WireframeModelLoadDirector : public ModelLoadDirector
{
public:
  WireframeModelLoadDirector();
  ~WireframeModelLoadDirector() = default;

  std::shared_ptr<BaseObject> load(const std::string& src_name) override;

private:
  void loadVertices();
  void loadEdges();
  void loadCenter();
};

#endif  // WIREFRAME_MODEL_LOAD_DIRECTOR_H
