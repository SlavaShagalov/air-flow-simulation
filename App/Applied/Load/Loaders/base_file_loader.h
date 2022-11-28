#ifndef BASE_FILE_LOADER_H
#define BASE_FILE_LOADER_H

#include <memory>
#include <fstream>

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <Domains/Applied/Primitives/Edge/edge.h>

class BaseFileLoader {
public:
  BaseFileLoader() = default;
  virtual ~BaseFileLoader() = default;

  virtual void openFile(const std::string& src_name) = 0;

  virtual void closeFile() = 0;

  virtual Vec3f loadVertex() = 0;

  virtual Edge loadEdge() = 0;

  virtual size_t loadCount() = 0;

protected:
  std::shared_ptr<std::ifstream> _srcFile;
};

#endif  // BASE_FILE_LOADER_H
