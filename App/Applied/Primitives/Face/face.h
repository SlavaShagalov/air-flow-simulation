#ifndef FACE_H
#define FACE_H

#include <cstddef>
#include <vector>

class Face {
public:
  Face() = default;
  Face(const Face &other) = default;

  Face(const std::vector<size_t> &pointsIndeces)
      : _pointsIndices(pointsIndeces) {}

  size_t size() const { return _pointsIndices.size(); }

  size_t vertex(const size_t &index) const { return _pointsIndices[index]; }
  size_t normal(const size_t &index) const { return _normalsIndices[index]; }

  void push_back(const size_t &vertexInd, const size_t &normalInd) {
    _pointsIndices.push_back(vertexInd);
    _normalsIndices.push_back(normalInd);
  }

  ~Face() = default;

private:
  std::vector<size_t> _pointsIndices;
  std::vector<size_t> _normalsIndices;
};

#endif // FACE_H
