#include "transform_visitor.h"

#include <Domains/Applied/SceneObjects/Model/polygonal_model.h>

#include <Domains/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>
#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <Domains/Applied/Primitives/Vector4D/vector_4d.hpp>

void TransformVisitor::visit(WireframeModel& model) {
  //  // transform vertices
  //  auto& vertices = model._components->getVertices();

  //  for (auto& vertex : vertices) {
  //    Matrix<double> curVertex = { { vertex.x(), vertex.y(), vertex.z(), 1 }
  //    };

  //    Matrix<double> transformedVertex = curVertex * (*_mtr);

  //    vertex = PointD_3D(transformedVertex[0][0], transformedVertex[0][1],
  //                       transformedVertex[0][2]);
  //  }

  //  // transform center
  //  auto& center = model.getCenter();
  //  Matrix<double> curCenter = { { center.x(), center.y(), center.z(), 1 } };

  //  Matrix<double> transformedCenter = curCenter * (*_mtr);

  //  center = PointD_3D(transformedCenter[0][0], transformedCenter[0][1],
  //                     transformedCenter[0][2]);
}

void TransformVisitor::visit(PolygonalModel& model) {
  // transform vertices
  auto& vertices = model._components->vertices();

  Vec4f transVec;

  for (auto& vertex : vertices) {
    transVec = (*_mtr) * Vec4f(vertex.x(), vertex.y(), vertex.z(), 1);
    vertex = Vec3f(transVec[0], transVec[1], transVec[2]);
  }

  //  // transform normals
  auto& normals = model._components->normals();

  for (auto& normal : normals) {
    transVec = (*_mtr) * Vec4f(normal.x(), normal.y(), normal.z(), 1);
    normal = Vec3f(transVec[0], transVec[1], transVec[2]);
  }

  // transform center
  auto& center = model._center;
  transVec = (*_mtr) * Vec4f(center.x(), center.y(), center.z(), 1);
  center = Vec3f(transVec[0], transVec[1], transVec[2]);
}

void TransformVisitor::visit(Camera& camera) {
  auto& eye = camera._eye;
  Vec4f transVec = (*_mtr) * Vec4f(eye.x(), eye.y(), eye.z(), 1);
  eye = Vec3f(transVec[0], transVec[1], transVec[2]);
}
