#ifndef ROTATE_MODEL_COMMAND_H
#define ROTATE_MODEL_COMMAND_H

#include "model_command.h"

#include <cstddef>

class RotateModelCommand : public ModelCommand {
public:
  RotateModelCommand() = delete;

  RotateModelCommand(std::shared_ptr<BaseObject> model, double ax, double ay,
                     double az)
      : _model(model), _ax(ax), _ay(ay), _az(az) {}

  ~RotateModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->rotateObject(_model, _ax, _ay, _az);
  }

private:
  std::shared_ptr<BaseObject> _model;
  double _ax, _ay, _az;
};

#endif  // ROTATE_MODEL_COMMAND_H
