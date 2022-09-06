#ifndef CAMERA_FILE_LOADER_H
#define CAMERA_FILE_LOADER_H

#include <memory>
#include <fstream>

#include <Domains/Applied/Load/Loaders/base_file_loader.h>

class CameraFileLoader : public BaseFileLoader {
public:
  CameraFileLoader() = default;
  ~CameraFileLoader() = default;

  void openFile(const std::string& srcFileName);

  void closeFile();

  Vec3f loadVertex();

  Edge loadEdge() { return Edge(0, 0); }

  size_t loadCount() { return 0; }
};

#endif  // CAMERA_FILE_LOADER_H
