#ifndef LOAD_MODEL_COMMAND_H
#define LOAD_MODEL_COMMAND_H

#include "model_command.h"

#include <string>

class LoadModelCommand : public ModelCommand {
private:
  std::shared_ptr<BaseObject>& _model;
  std::string _fileName;
  size_t _directorId;

public:
  LoadModelCommand() = delete;

  LoadModelCommand(std::shared_ptr<BaseObject>& model,
                   const std::string& fileName)
      : _model(model), _fileName(fileName), _directorId(2) {
    qDebug() << "LoadModelCommand created";
  }

  ~LoadModelCommand() override = default;

  virtual void execute(std::shared_ptr<MainManager> mainManager) override {
    qDebug() << "LoadModelCommand::execute()";
    _model = mainManager->loadObject(_fileName, _directorId);
  }
};

#endif  // LOAD_MODEL_COMMAND_H
