#ifndef ADD_MODEL_COMMAND_H
#define ADD_MODEL_COMMAND_H

#include "model_command.h"

#include <memory>

class AddModelCommand : public ModelCommand {
public:
  AddModelCommand() = delete;

  AddModelCommand(std::shared_ptr<BaseObject> model) : _model(model) {}

  ~AddModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->addObject(_model);
  }

private:
  std::shared_ptr<BaseObject> _model;
};

#endif  // ADD_MODEL_COMMAND_H
