#include "camera_file_loader.h"

#include <Domains/Applied/Exceptions/load_exceptions.h>

void CameraFileLoader::openFile(const std::string& srcFileName) {
  _srcFile = std::make_shared<std::ifstream>(srcFileName);

  if (!*(_srcFile))
    throw FileOpenError(__FILE__, __LINE__, "could not open camera-file");
}

Vec3f CameraFileLoader::loadVertex() {
  double x, y, z;

  if (!(*(_srcFile) >> x >> y >> z))
    throw FileFormatError(__FILE__, __LINE__, "bad format of camera-file");

  return Vec3f(x, y, z);
}

void CameraFileLoader::closeFile() { _srcFile->close(); }
