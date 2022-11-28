#include "wireframe_model_file_loader.h"

#include <Domains/Applied/Exceptions/load_exceptions.h>

void WireframeModelFileLoader::openFile(const std::string& src_name) {
  _srcFile = std::make_shared<std::ifstream>(src_name);
  if (!*(_srcFile))
    throw FileOpenError(__FILE__, __LINE__, "could not open model-file");
}

void WireframeModelFileLoader::closeFile() { _srcFile->close(); }

Vec3f WireframeModelFileLoader::loadVertex() {
  double x, y, z;
  if (!(*(_srcFile) >> x >> y >> z))
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return Vec3f(x, y, z);
}

Edge WireframeModelFileLoader::loadEdge() {
  size_t p1_id, p2_id;
  if (!(*(_srcFile) >> p1_id >> p2_id))
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return Edge(p1_id, p2_id);
}

size_t WireframeModelFileLoader::loadCount() {
  size_t count = 0;

  *(_srcFile) >> count;
  if (count <= 1)
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return count;
}
