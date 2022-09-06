#ifndef SOLUTION_DRAWER_FACTORY_H
#define SOLUTION_DRAWER_FACTORY_H

#include "drawer.h"

#include <map>
#include <string>

class QGraphicsScene;

class SolutionDrawerFactory {
  using CallBackMap = std::map<std::string, std::shared_ptr<DrawerFactory>>;

public:
  SolutionDrawerFactory() = default;

  template <typename Tprod, typename... Args>
  bool registration(std::string id, Args... args) {
    return callbacks.emplace(id, std::make_shared<Tprod>(args...)).second;
  }

  bool check(std::string id) { return callbacks.erase(id) == 1; }

  std::shared_ptr<DrawerFactory> createFactory(std::string id);

private:
  CallBackMap callbacks{};
};

#endif  // SOLUTION_DRAWER_FACTORY_H
