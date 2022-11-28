#ifndef CAMERA_EXCEPTIONS_H
#define CAMERA_EXCEPTIONS_H

#include "base_exception.h"

class LastCameraRemoveError : public BaseException {
public:
  explicit LastCameraRemoveError(const string& file, const int line,
                                 const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "LastCameraRemoveError: " + this->error_msg;
  }
};

#endif  // CAMERA_EXCEPTIONS_H
