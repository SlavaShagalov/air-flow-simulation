#ifndef MOVE_MODEL_COMMAND_H
#define MOVE_MODEL_COMMAND_H

#include "model_command.h"

#include <cstddef>

class MoveModelCommand : public ModelCommand {
public:
  MoveModelCommand() = delete;

  MoveModelCommand(std::shared_ptr<BaseObject> model, const double &dx,
                   const double &dy, const double &dz)
      : model(model), _dx(dx), _dy(dy), _dz(dz) {}

  ~MoveModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->moveObject(model, _dx, _dy, _dz);
  }

private:
  std::shared_ptr<BaseObject> model;

  double _dx, _dy, _dz;
};

#endif  // MOVE_MODEL_COMMAND_H
