#ifndef BASE_EXCEPTION_H
#define BASE_EXCEPTION_H

#include <exception>
#include <ctime>
#include <string>

using string = std::string;

class BaseException : public std::exception {
protected:
  string error_msg;

public:
  BaseException(const string& file, const int line,
                const string& err_msg = "No error message") {
    time_t curr_time = time(nullptr);
    auto local_time = localtime(&curr_time);
    this->error_msg = err_msg + "\nFile: " + file +
                      "\nLine: " + std::to_string(line) +
                      "\nTime: " + asctime(local_time);
  }

  const char* what() const noexcept override { return this->error_msg.c_str(); }
};

#endif  // BASE_EXCEPTION_H
