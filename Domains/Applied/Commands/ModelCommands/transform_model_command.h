#ifndef TRANSFORM_MODEL_COMMAND_H
#define TRANSFORM_MODEL_COMMAND_H

#include "model_command.h"

#include <cstddef>

class TransformModelCommand : public ModelCommand {
public:
  TransformModelCommand() = delete;

  TransformModelCommand(std::shared_ptr<BaseObject> model,
                        const Mtr4x4f& mtr)
      : _model(model), _mtr(mtr) {}

  ~TransformModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->transformObject(_model, _mtr);
  }

private:
  std::shared_ptr<BaseObject> _model;
  Mtr4x4f _mtr;
};

#endif  // TRANSFORM_MODEL_COMMAND_H
