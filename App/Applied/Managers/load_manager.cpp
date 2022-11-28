#include <App/Applied/Managers/load_manager.h>

#include <QDebug>

#include <App/Applied/Solutions/load_director_solution.h>

std::shared_ptr<BaseObject> LoadManager::load(const std::string& fileName,
                                              size_t directorId) {
//  qDebug() << "LoadManager::load()";

  LoadDirectorSolution solution;
  _loadDirector = solution.createDirector(directorId);
//  qDebug() << "Success director create";

  return _loadDirector->load(fileName);
}
