#ifndef REMOVE_MODEL_COMMAND_H
#define REMOVE_MODEL_COMMAND_H

#include "model_command.h"

#include <cstddef>

class RemoveModelCommand : public ModelCommand {
public:
  RemoveModelCommand() = delete;

  explicit RemoveModelCommand(size_t modelId) : _modelId(modelId) {}

  ~RemoveModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->removeObject(_modelId);
  }

private:
  size_t _modelId;
};

#endif  // REMOVE_MODEL_COMMAND_H
