#ifndef FILE_MODEL_BUILDER_H
#define FILE_MODEL_BUILDER_H

#include "model_builder.h"

class FileModelBuilder : public ModelBuilder {
public:
  FileModelBuilder() = default;

  virtual ~FileModelBuilder() = default;

  virtual void reset() = 0;

  virtual std::shared_ptr<BaseObject> get() = 0;

  virtual void buildVertex(Vec3f) = 0;

  virtual void buildEdge(Edge) = 0;

  virtual void buildCenter(Vec3f) = 0;
};

#endif  // FILE_MODEL_BUILDER_H
