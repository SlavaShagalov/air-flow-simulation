#ifndef MODEL_LOAD_DIRECTOR_H
#define MODEL_LOAD_DIRECTOR_H

#include <App/Applied/Load/Directors/base_load_director.h>

class ModelLoadDirector : public BaseLoadDirector {
public:
  ModelLoadDirector() = default;
  ~ModelLoadDirector() = default;

  virtual std::shared_ptr<BaseObject> load(const std::string& src_name) = 0;
};

#endif  // MODEL_LOAD_DIRECTOR_H
