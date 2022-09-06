#ifndef GET_OBJECT_COMMAND_H
#define GET_OBJECT_COMMAND_H

#include "scene_command.h"

#include <cstddef>

class GetSceneObjectCommand : public SceneCommand {
public:
  GetSceneObjectCommand() = delete;

  explicit GetSceneObjectCommand(std::shared_ptr<BaseObject>& object, size_t id)
      : object(object), id(id) {}

  ~GetSceneObjectCommand() override = default;

  void execute(std::shared_ptr<MainManager> mainManager) override {
    object = mainManager->getSceneObject(id);
  }

private:
  std::shared_ptr<BaseObject>& object;
  size_t id;
};
#endif  // GET_OBJECT_COMMAND_H
