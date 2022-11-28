#ifndef WIREFRAME_MODEL_COMPONENTS_H
#define WIREFRAME_MODEL_COMPONENTS_H

#include <vector>

#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <App/Applied/Primitives/Edge/edge.h>

class WireframeModelComponents {
public:
  friend class DrawVisitor;

  WireframeModelComponents() = default;

  WireframeModelComponents(std::vector<Vec3f>& points, std::vector<Edge>& edges)
      : _vertices(points), _edges(edges) {}

  ~WireframeModelComponents() = default;

  void setVertices(const std::vector<Vec3f>& value) { _vertices = value; }
  void setEdges(const std::vector<Edge>& value) { _edges = value; }

  std::vector<Vec3f>& vertices() { return _vertices; }
  std::vector<Vec3f> vertices() const { return _vertices; }
  std::vector<Edge>& edges() { return _edges; }
  std::vector<Edge> edges() const { return _edges; }

  void addVertex(const Vec3f& vertex) { _vertices.push_back(vertex); }
  void addEdge(const Edge& edge) { _edges.push_back(edge); }

private:
  std::vector<Vec3f> _vertices;
  std::vector<Edge> _edges;
};

#endif  // WIREFRAME_MODEL_COMPONENTS_H
