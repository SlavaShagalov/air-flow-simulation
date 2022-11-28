#ifndef MANAGER_CREATOR_H
#define MANAGER_CREATOR_H

#include <iostream>
#include <memory>

using namespace std;

template <typename ConManager>
class ManagerCreator {
public:
  shared_ptr<ConManager> getManager() {
    if (!_manager) {
      _manager = createManager();
    }

    return _manager;
  }

protected:
  virtual shared_ptr<ConManager> createManager() {
    return shared_ptr<ConManager>(make_shared<ConManager>());
  }

private:
  shared_ptr<ConManager> _manager = nullptr;
};
#endif  // MANAGER_CREATOR_H
