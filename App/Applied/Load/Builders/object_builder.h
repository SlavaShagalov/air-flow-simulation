#ifndef OBJECT_BUILDER_H
#define OBJECT_BUILDER_H

#include <string>

#include <Domains/Applied/SceneObjects/base_object.h>

#include <Domains/Applied/Primitives/Edge/edge.h>

class ObjectBuilder {
public:
  ObjectBuilder() = default;

  virtual ~ObjectBuilder() = default;

  virtual void reset() = 0;

  virtual std::shared_ptr<BaseObject> get() = 0;

  virtual void buildVertex(Vec3f) = 0;

  virtual void buildEdge(Edge) = 0;

  virtual void buildCenter(Vec3f) = 0;

  virtual void buildPosition(Vec3f) = 0;
};

#endif  // OBJECT_BUILDER_H
