#ifndef LOAD_EXCEPTIONS_H
#define LOAD_EXCEPTIONS_H

#include "base_exception.h"

class FileError : public BaseException {
public:
  explicit FileError(const string& file, const int line, const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "FileError: " + this->error_msg;
  }
};

class BuildError : public BaseException {
public:
  explicit BuildError(const string& file, const int line, const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "BuildError: " + this->error_msg;
  }
};

class NoDirectorError : public BaseException {
public:
  explicit NoDirectorError(const string& file, const int line,
                           const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "NoDirectorError: " + this->error_msg;
  }
};

class ConfigError : public BaseException {
public:
  explicit ConfigError(const string& file, const int line, const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "ConfigurationError: " + this->error_msg;
  }
};

class NoCameraError : public BaseException {
public:
  explicit NoCameraError(const string& file, const int line, const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "NoCameraError: " + this->error_msg;
  }
};

class FileOpenError : public BaseException {
public:
  explicit FileOpenError(const string& file, const int line, const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "FileOpenError: " + this->error_msg;
  }
};

class FileFormatError : public BaseException {
public:
  explicit FileFormatError(const string& file, const int line,
                           const string& msg)
      : BaseException(file, line, msg) {
    this->error_msg = "FileFormatError: " + this->error_msg;
  }
};

#endif  // LOAD_EXCEPTIONS_H
