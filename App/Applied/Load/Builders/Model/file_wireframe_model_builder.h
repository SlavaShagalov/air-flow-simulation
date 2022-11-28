#ifndef FILE_WIREFRAME_MODEL_BUILDER_H
#define FILE_WIREFRAME_MODEL_BUILDER_H

#include "file_model_builder.h"

#include <Domains/Applied/SceneObjects/base_object.h>

#include <Domains/Applied/Primitives/Edge/edge.h>

#include <Domains/Applied/SceneObjects/Model/wireframe_model_components.h>

class FileWireframeModelBuilder : public FileModelBuilder {
public:
  FileWireframeModelBuilder();

  ~FileWireframeModelBuilder() = default;

  void reset();

  std::shared_ptr<BaseObject> get();

  void buildVertex(Vec3f);

  void buildEdge(Edge);

  void buildCenter(Vec3f);

  virtual void buildPosition(Vec3f) {}

private:
  std::shared_ptr<WireframeModelComponents> _modelComponents;
  Vec3f _center;
};

#endif  // FILE_WIREFRAME_MODEL_BUILDER_H
