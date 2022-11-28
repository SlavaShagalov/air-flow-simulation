#ifndef ADD_PARTICLE_SYSTEM_COMMAND_H
#define ADD_PARTICLE_SYSTEM_COMMAND_H

#include <memory>

#include <App/Applied/Commands/base_command.h>

class AddParticleSystemCommand : public BaseCommand {
public:
  AddParticleSystemCommand() = delete;

  AddParticleSystemCommand(std::shared_ptr<BaseObject> psys) : _psys(psys) {}

  ~AddParticleSystemCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->addObject(_psys);
  }

private:
  std::shared_ptr<BaseObject> _psys;
};

#endif  // ADD_PARTICLE_SYSTEM_COMMAND_H
