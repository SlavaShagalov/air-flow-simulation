#ifndef REMOVE_PARTICLE_SYSTEM_COMMAND_H
#define REMOVE_PARTICLE_SYSTEM_COMMAND_H

#include <cstddef>

#include <Domains/Applied/Commands/base_command.h>

class RemoveParticleSystemCommand : public BaseCommand {
public:
  RemoveParticleSystemCommand() = delete;

  explicit RemoveParticleSystemCommand(size_t objectId) : _objectId(objectId) {}

  ~RemoveParticleSystemCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    mainManager->removeObject(_objectId);
  }

private:
  size_t _objectId;
};

#endif  // REMOVE_PARTICLE_SYSTEM_COMMAND_H
