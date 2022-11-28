#ifndef MODEL_BUILDER_H
#define MODEL_BUILDER_H

#include <Domains/Applied/Load/Builders/object_builder.h>

class ModelBuilder : public ObjectBuilder {
public:
  ModelBuilder() = default;

  virtual ~ModelBuilder() = default;

  virtual void reset() = 0;

  virtual std::shared_ptr<BaseObject> get() = 0;

  virtual void buildVertex(Vec3f) = 0;

  virtual void buildEdge(Edge) = 0;

  virtual void buildCenter(Vec3f) = 0;
};

#endif  // MODEL_BUILDER_H
