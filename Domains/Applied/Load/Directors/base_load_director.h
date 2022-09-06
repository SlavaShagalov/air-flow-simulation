#ifndef BASE_LOAD_DIRECTOR_H
#define BASE_LOAD_DIRECTOR_H

#include <memory>
#include <utility>

#include <Domains/Applied/SceneObjects/base_object.h>

#include <Domains/Applied/Load/Builders/object_builder.h>

#include <Domains/Applied/Load/Loaders/base_file_loader.h>

class BaseLoadDirector {
public:
  BaseLoadDirector() = default;
  ~BaseLoadDirector() = default;

  virtual std::shared_ptr<BaseObject> load(const std::string& name) = 0;

protected:
  std::shared_ptr<ObjectBuilder> _builder;
  std::shared_ptr<BaseFileLoader> _loader;
};

#endif  // BASE_LOAD_DIRECTOR_H
