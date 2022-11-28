#ifndef ADD_LIGHT_COMMAND_H
#define ADD_LIGHT_COMMAND_H

#include <memory>

#include <App/Applied/Commands/base_command.h>

class AddLightCommand : public BaseCommand {
public:
  AddLightCommand() = delete;

  AddLightCommand(const std::shared_ptr<Light> light) : _light(light) {}

  AddLightCommand(const Vec3f& dir) { _light = std::make_shared<Light>(dir); }

  ~AddLightCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->addLight(_light);
  }

private:
  std::shared_ptr<Light> _light;
};

#endif  // ADD_LIGHT_COMMAND_H
