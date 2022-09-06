#ifndef LOAD_DIRECTOR_SOLUTION_H
#define LOAD_DIRECTOR_SOLUTION_H

#include <map>
#include <string>

#include <Domains/Applied/Load/Directors/base_load_director.h>

class LoadDirectorSolution {
  using CallBackMap = std::map<size_t, std::shared_ptr<BaseLoadDirector>>;

public:
  LoadDirectorSolution() { initId(); }

  template <typename Tprod>
  bool registration(size_t id);

  void initId();

  bool check(size_t id) { return callbacks.erase(id) == 1; }

  std::shared_ptr<BaseLoadDirector> createDirector(size_t id);

private:
  CallBackMap callbacks{};
};

#endif  // LOAD_DIRECTOR_SOLUTION_H
