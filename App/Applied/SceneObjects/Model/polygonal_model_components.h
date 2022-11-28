#ifndef POLYGONAL_MODEL_COMPONENTS_H
#define POLYGONAL_MODEL_COMPONENTS_H

#include <vector>

#include "App/Applied/Primitives/Vector3D/vector_3d.hpp"
#include <App/Applied/Primitives/Face/face.h>

class PolygonalModelComponents {
public:
  PolygonalModelComponents() = default;

  PolygonalModelComponents(const std::vector<Vec3f>& points,
                           const std::vector<Face>& faces)
      : _vertices(points), _faces(faces) {}

  ~PolygonalModelComponents() = default;

  void setVertices(const std::vector<Vec3f>& value) { _vertices = value; }
  void setFaces(const std::vector<Face>& value) { _faces = value; }

  std::vector<Vec3f> vertices() const { return _vertices; }
  std::vector<Vec3f>& vertices() { return _vertices; }
  std::vector<Face> faces() const { return _faces; }
  std::vector<Face>& faces() { return _faces; }
  std::vector<Vec3f> normals() const { return _normals; }
  std::vector<Vec3f>& normals() { return _normals; }

  //  void addTexture(const Point2D_D& texture) { _textures.push_back(texture);
  //  }
  void addNormal(const Vec3f& normal) { _normals.push_back(normal); }
  void addVertex(const Vec3f& vertex) { _vertices.push_back(vertex); }
  void addFace(const Face& face) { _faces.push_back(face); }

private:
  std::vector<Vec3f> _vertices;
  std::vector<Vec3f> _normals;
  //  std::vector<Point2D_D> _textures;
  std::vector<Face> _faces;

  //  std::shared_ptr<Image> _textureImage;
};

#endif  // POLYGONAL_MODEL_COMPONENTS_H
