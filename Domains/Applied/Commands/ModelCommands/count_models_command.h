#ifndef COUNT_MODELS_COMMAND_H
#define COUNT_MODELS_COMMAND_H

#include "model_command.h"

#include <memory>
#include <cstddef>

class CountModelsCommand : public ModelCommand {
public:
  CountModelsCommand() = delete;

  CountModelsCommand(size_t& count) : _count(count) {}

  ~CountModelsCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    //    _count = mainManager->countModels();
  }

private:
  std::size_t& _count;
};

#endif  // COUNT_MODELS_COMMAND_H
