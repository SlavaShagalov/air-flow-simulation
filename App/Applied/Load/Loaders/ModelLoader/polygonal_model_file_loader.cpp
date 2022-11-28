#include "polygonal_model_file_loader.h"

#include <App/Applied/Exceptions/load_exceptions.h>

void PolygonalModelObjFileLoader::openFile(const std::string& src_name) {
  _srcFile = std::make_shared<std::ifstream>(src_name);
  if (!*(_srcFile))
    throw FileOpenError(__FILE__, __LINE__, "could not open model-file");
}

void PolygonalModelObjFileLoader::closeFile() { _srcFile->close(); }

Vec3f PolygonalModelObjFileLoader::loadVertex() {
  double x, y, z;
  if (!(*(_srcFile) >> x >> y >> z))
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return Vec3f(x, y, z);
}

Edge PolygonalModelObjFileLoader::loadEdge() {
  size_t p1_id, p2_id;
  if (!(*(_srcFile) >> p1_id >> p2_id))
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return Edge(p1_id, p2_id);
}

size_t PolygonalModelObjFileLoader::loadCount() {
  size_t count = 0;

  *(_srcFile) >> count;
  if (count <= 1)
    throw FileFormatError(__FILE__, __LINE__, "invalid model-file format");

  return count;
}
