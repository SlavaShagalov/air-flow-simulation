#ifndef EDGE_H
#define EDGE_H

#include <cstddef>

class Edge {
private:
  size_t _v1Index;
  size_t _v2Index;

public:
  Edge() = default;
  Edge(const Edge& other) = default;
  //  Edge(Edge&& other) = default;

  Edge(const size_t v1Index, const size_t v2Index)
      : _v1Index(v1Index), _v2Index(v2Index) {}

  ~Edge() = default;

  size_t first() const { return _v1Index; }
  size_t second() const { return _v2Index; }

  size_t operator[](size_t index) const {
    return index == 0 ? _v1Index : _v2Index;
  }

  void setFirst(const size_t vIndex) { _v1Index = vIndex; }
  void setSecond(const size_t vIndex) { _v2Index = vIndex; }
};

#endif  // EDGE_H
