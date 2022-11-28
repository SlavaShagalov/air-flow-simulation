#ifndef LOAD_MANAGER_H
#define LOAD_MANAGER_H

#include "base_manager.h"

#include <memory>

#include <Domains/Applied/SceneObjects/base_object.h>

#include <Domains/Applied/Load/Directors/base_load_director.h>

class LoadManager : public BaseManager {
public:
  LoadManager() = default;

  LoadManager(const LoadManager& other) = delete;
  LoadManager(LoadManager&& other) = delete;

  LoadManager& operator=(const LoadManager&) = delete;
  LoadManager& operator=(LoadManager&&) = delete;

  ~LoadManager() override = default;

  virtual std::shared_ptr<BaseObject> load(const std::string& name,
                                           size_t director_id);

private:
  std::shared_ptr<BaseLoadDirector> _loadDirector;
};

#endif  // LOAD_MANAGER_H
