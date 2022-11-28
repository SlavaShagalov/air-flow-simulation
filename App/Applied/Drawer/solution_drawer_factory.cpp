#include "solution_drawer_factory.h"

#include <App/Applied/Exceptions/load_exceptions.h>

std::shared_ptr<DrawerFactory>
SolutionDrawerFactory::createFactory(std::string id) {
  CallBackMap::const_iterator it = callbacks.find(id);

  if (it == callbacks.end()) {
    throw ConfigError(__FILE__, __LINE__, "wrong id");
  }

  return it->second;
}
