#ifndef LOAD_POLYGONAL_MODEL_COMMAND_H
#define LOAD_POLYGONAL_MODEL_COMMAND_H

#include "model_command.h"

#include <string>

class LoadPolygonalModelCommand : public ModelCommand {
private:
  std::shared_ptr<BaseObject>& _model;
  std::string _fileName;
  size_t _directorId;

public:
  LoadPolygonalModelCommand() = delete;

  LoadPolygonalModelCommand(std::shared_ptr<BaseObject>& model,
                            const std::string& fileName)
      : _model(model), _fileName(fileName), _directorId(3) {
//    qDebug() << "LoadPolygModelCommand created";
  }

  ~LoadPolygonalModelCommand() override = default;

  virtual void execute(std::shared_ptr<MainManager> mainManager) override {
//    qDebug() << "LoadPolygModelCommand::execute()";
    _model = mainManager->loadObject(_fileName, _directorId);
  }
};

#endif  // LOAD_POLYGONAL_MODEL_COMMAND_H
