#ifndef POLYGONAL_MODEL_FILE_LOADER_H
#define POLYGONAL_MODEL_FILE_LOADER_H

#include <memory>
#include <fstream>

#include <Domains/Applied/Load/Loaders/base_file_loader.h>

class PolygonalModelObjFileLoader : public BaseFileLoader {
public:
  PolygonalModelObjFileLoader() = default;
  ~PolygonalModelObjFileLoader() = default;

  void openFile(const std::string& srcFileName);
  void closeFile();

  Vec3f loadVertex();
  Edge loadEdge();
  size_t loadCount();
};

#endif  // POLYGONAL_MODEL_FILE_LOADER_H
