#ifndef BASE_COMMAND_H
#define BASE_COMMAND_H

#include <memory>

#include <Domains/Applied/Managers/main_manager.h>

class Facade;

class BaseCommand {
public:
  BaseCommand() = default;

  virtual ~BaseCommand() = default;

  virtual void execute(std::shared_ptr<MainManager> controller) = 0;
};

#endif  // BASE_COMMAND_H
