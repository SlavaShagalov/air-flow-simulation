cmake -B ./Build -DCMAKE_BUILD_TYPE=Debug \
  -DBUILD_TESTS=ON \
  -DUSE_SANITIZER_FLAGS=ON \
  -DUSE_COVERAGE_FLAGS=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build Build
