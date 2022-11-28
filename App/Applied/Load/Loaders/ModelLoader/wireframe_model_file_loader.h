#ifndef WIREFRAME_MODEL_FILE_LOADER_H
#define WIREFRAME_MODEL_FILE_LOADER_H

#include <memory>
#include <fstream>

#include <App/Applied/Load/Loaders/base_file_loader.h>

class WireframeModelFileLoader : public BaseFileLoader {
public:
  WireframeModelFileLoader() = default;
  ~WireframeModelFileLoader() = default;

  void openFile(const std::string& srcFileName);
  void closeFile();

  Vec3f loadVertex();
  Edge loadEdge();
  size_t loadCount();
};

#endif  // WIREFRAME_MODEL_FILE_LOADER_H
