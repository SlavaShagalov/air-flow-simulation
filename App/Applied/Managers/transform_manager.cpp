
#include "transform_manager.h"

#include <cmath>

#include <QDebug>

#include <Domains/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>

#include <Domains/Applied/Visitors/transform_visitor.h>

void TransformManager::moveObject(const std::shared_ptr<BaseObject> &object,
                                  const double &dx, const double &dy,
                                  const double &dz) {
  //  qDebug() << dx << "; " << dy << "; " << dz;

  const Mtr4x4f mtr(1, 0, 0, dx, //
                    0, 1, 0, dy, //
                    0, 0, 1, dz, //
                    0, 0, 0, 1);

  auto visitor = TransformVisitor(mtr);
  object->accept(visitor);
}

void TransformManager::scaleObject(const std::shared_ptr<BaseObject> &object,
                                   const double &kx, const double &ky,
                                   const double &kz) {
  auto objCenter = object->center();
  moveObject(object, -objCenter.x(), -objCenter.y(), -objCenter.z());

  const Mtr4x4f mtr(kx, 0, 0, 0, //
                    0, ky, 0, 0, //
                    0, 0, kz, 0, //
                    0, 0, 0, 1);

  auto visitor = TransformVisitor(mtr);
  object->accept(visitor);

  moveObject(object, objCenter.x(), objCenter.y(), objCenter.z());
}

void TransformManager::rotateObject(const std::shared_ptr<BaseObject> &object,
                                    const double &ax, const double &ay,
                                    const double &az) {
  const auto &objCenter = object->center();
  moveObject(object, -objCenter.x(), -objCenter.y(), -objCenter.z());

  const Mtr4x4f mtrOX(1, 0, 0, 0,              //
                      0, cos(ax), -sin(ax), 0, //
                      0, sin(ax), cos(ax), 0,  //
                      0, 0, 0, 1);

  const Mtr4x4f mtrOY(cos(ay), 0, -sin(ay), 0,  //
                      0, 1, 0, 0,              //
                      sin(ay), 0, cos(ay), 0, //
                      0, 0, 0, 1);

  const Mtr4x4f mtrOZ(cos(az), -sin(az), 0, 0, //
                      sin(az), cos(az), 0, 0,  //
                      0, 0, 1, 0,              //
                      0, 0, 0, 1);

  auto visitor = TransformVisitor(mtrOX * mtrOY * mtrOZ);
  object->accept(visitor);

  moveObject(object, objCenter.x(), objCenter.y(), objCenter.z());
}

void TransformManager::transformObject(
    const std::shared_ptr<BaseObject> &object, const Mtr4x4f &mtr) {
  auto visitor = TransformVisitor(mtr);
  object->accept(visitor);
}
