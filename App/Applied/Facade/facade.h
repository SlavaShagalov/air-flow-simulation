#ifndef FACADE_H
#define FACADE_H

#include <memory>

#include <Domains/Applied/Managers/main_manager.h>
#include <Domains/Applied/Commands/base_command.h>

class Facade {
private:
  std::shared_ptr<MainManager> _mainManager;

public:
  Facade() { _mainManager = MainManager::instance(); }
  Facade(const Facade& other) = delete;
  Facade(Facade&& other) = delete;

  ~Facade() = default;

  void executeCommand(BaseCommand& command) {
    //    qDebug() << "Facade::executeCommand()";
    command.execute(_mainManager);
  }
};
#endif  // FACADE_H
