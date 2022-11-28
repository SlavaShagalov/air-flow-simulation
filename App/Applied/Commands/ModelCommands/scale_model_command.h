#ifndef SCALE_MODEL_COMMAND_H
#define SCALE_MODEL_COMMAND_H

#include "model_command.h"

#include <cstddef>

class ScaleModelCommand : public ModelCommand {
public:
  ScaleModelCommand() = delete;

  ScaleModelCommand(std::shared_ptr<BaseObject> model, double kx, double ky,
                    double kz)
      : model(model), kx(kx), ky(ky), kz(kz) {}

  ~ScaleModelCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->scaleObject(model, kx, ky, kz);
  }

private:
  std::shared_ptr<BaseObject> model;

  double kx, ky, kz;
};

#endif  // SCALE_MODEL_COMMAND_H
