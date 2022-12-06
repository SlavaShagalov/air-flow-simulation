#include "draw_visitor.h"

void DrawVisitor::visit(const WireframeModel& model) {
  //  std::cout << "[DBG]: DrawVisitor::visit(const WireframeModel&)\n";
  // compute matrices
  const Mtr4x4f modelView = Mtr4x4f::lookAt(_camera->eye(), _camera->center(), _camera->up());

  const int width = _drawer->width(), height = _drawer->height();
  const Mtr4x4f viewport = Mtr4x4f::viewport(0, 0, width, height);
  const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
                                         0, -1, 0, 0,  //
                                         0, 0, 1, 0,   //
                                         0, 0, 0, 1);
  const Mtr4x4f mtr = viewport * flipVertically * modelView;

  // get model data
  auto vertices = model._components->vertices();
  const auto& edges = model._components->edges();

  // mult matrix
  Vec4f tvec;
  for (auto& v : vertices) {
    tvec = mtr * Vec4f(v.x(), v.y(), v.z(), 1);
    v = Vec3f(tvec.x() / tvec.w(), tvec.y() / tvec.w(), tvec.z() / tvec.w());
  }

  // draw
  for (const auto& edge : edges)
    _drawer->drawLine(vertices[edge[0]], vertices[edge[1]], model._color);
}

void DrawVisitor::visit(const PolygonalModel& model) {
  //  qDebug() << "DrawVisitor::visit(const PolygonalModel& model)";
  switch (_mode) {
    case WIREFRAME:
      wireframeDraw(model);
      break;
    case SIMPLE:
      simpleDraw(model);
      break;
    case GOURAND:
      gourandDraw(model);
      break;
    default:
      break;
  }
}

// Wireframe shading
void DrawVisitor::wireframeDraw(const PolygonalModel& model) {
  // input data
  const auto& vertices = model._components->vertices();
  const auto& faces = model._components->faces();

  // matrix
  const Mtr4x4f modelView = Mtr4x4f::lookAt(_camera->eye(), Vec3f(_camera->center()), _camera->up());

  const int width = _drawer->width(), height = _drawer->height();
  const Mtr4x4f viewport = Mtr4x4f::viewport(0, 0, width, height);

  const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
                                         0, -1, 0, 0,  //
                                         0, 0, 1, 0,   //
                                         0, 0, 0, 1);
  const Mtr4x4f mtr = viewport * flipVertically * modelView;

  Vec3f v1, v2, v3, v4;
  Vec4f trV;

  for (const auto& face : faces) {
    // matrix mult
    v1 = vertices[face.vertex(0)];
    v2 = vertices[face.vertex(1)];
    v3 = vertices[face.vertex(2)];

    trV = mtr * Vec4f(v1.x(), v1.y(), v1.z(), 1);
    v1 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v2.x(), v2.y(), v2.z(), 1);
    v2 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v3.x(), v3.y(), v3.z(), 1);
    v3 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());

    _drawer->drawLine(v1, v2);
    _drawer->drawLine(v2, v3);

    // draw
    if (face.size() == 3) {
      _drawer->drawLine(v3, v1);
    } else if (face.size() == 4) {
      v4 = vertices[face.vertex(3)];
      trV = mtr * Vec4f(v4.x(), v4.y(), v4.z(), 1);
      v4 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());

      _drawer->drawLine(v3, v4);
      _drawer->drawLine(v4, v1);
    }
  }
}

// Simple shading
void DrawVisitor::simpleDraw(const PolygonalModel& model) {
  // get model data
  const auto& vertices = model._components->vertices();
  const auto& faces = model._components->faces();

  // compute matrices
  const Mtr4x4f modelView = Mtr4x4f::lookAt(_camera->eye(), _camera->center(), _camera->up());
  const int width = _drawer->width(), height = _drawer->height();
  const Mtr4x4f viewport = Mtr4x4f::viewport(0, 0, width, height);
  const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
                                         0, -1, 0, 0,  //
                                         0, 0, 1, 0,   //
                                         0, 0, 0, 1);
  const Mtr4x4f mtr = viewport * flipVertically * modelView;

  // draw
  Vec3f light = (_camera->center() - _camera->eye()).normalize();
  Vec3f normal, v1, v2, v3;
  Vec4f trV;
  float intensity;
  for (const auto& face : faces) {
    v1 = vertices[face.vertex(0)];
    v2 = vertices[face.vertex(1)];
    v3 = vertices[face.vertex(2)];

    // compute intensity
    normal = Vec3f::crossProduct(v3 - v1, v2 - v1).normalize();
    intensity = Vec3f::dotProduct(normal, light);
    //    Vec3f::dotProduct(normal, _lights[0]->dir());  // TODO: some lights

    // mult matrix
    trV = mtr * Vec4f(v1.x(), v1.y(), v1.z(), 1);
    v1 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v2.x(), v2.y(), v2.z(), 1);
    v2 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v3.x(), v3.y(), v3.z(), 1);
    v3 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());

    // draw
    if (intensity > 0) {
      //      if (face.size() == 3) {
      _drawer->drawTriangle(v1, v2, v3, Color(255 * intensity, 255 * intensity, 255 * intensity, 255));

      //      }
      //      else if (face.size() == 4) {
      //        v4 = vertices[face.vertex(3)];
      //        trV = mtr * Vec4f(v4.x(), v4.y(), v4.z(), 1);
      //        v4 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() /
      //        trV.w());

      //        _drawer->drawQuadrangle(
      //        v1, v2, v3, v4,
      //        Color(255 * intensity, 255 * intensity, 255 * intensity, 255));
      //      }
    }
  }
}

// Gourand shading
void DrawVisitor::gourandDraw(const PolygonalModel& model) {
  // get model data
  const auto& vertices = model._components->vertices();
  const auto& faces = model._components->faces();
  const auto& normals = model._components->normals();

  Vec3f light = (_camera->eye() - _camera->center()).normalize();
  //  Vec3f light = _lights[0]->dir().normalize();

  // matrix
  const Mtr4x4f modelView = Mtr4x4f::lookAt(_camera->eye(), _camera->center(), _camera->up());
  const int width = _drawer->width(), height = _drawer->height();
  const Mtr4x4f viewport = Mtr4x4f::viewport(0, 0, width, height);
  const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
                                         0, -1, 0, 0,  //
                                         0, 0, 1, 0,   //
                                         0, 0, 0, 1);
  const Mtr4x4f mtr = viewport * flipVertically * modelView;

  Vec3f normal, v1, v2, v3;
  float i1, i2, i3;
  Vec4f trV;

  for (const auto& face : faces) {
    v1 = vertices[face.vertex(0)];
    v2 = vertices[face.vertex(1)];
    v3 = vertices[face.vertex(2)];

    // matrix mult
    trV = mtr * Vec4f(v1.x(), v1.y(), v1.z(), 1);
    v1 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v2.x(), v2.y(), v2.z(), 1);
    v2 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());
    trV = mtr * Vec4f(v3.x(), v3.y(), v3.z(), 1);
    v3 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());

    // compute intensity
    normal = normals[face.normal(0)];
    normal.normalize();
    i1 = Vec3f::dotProduct(normal, light);
    normal = normals[face.normal(1)];
    normal.normalize();
    i2 = Vec3f::dotProduct(normal, light);
    normal = normals[face.normal(2)];
    normal.normalize();
    i3 = Vec3f::dotProduct(normal, light);

    // draw
    //    if (face.size() == 3) {
    _drawer->drawTriangle(v1, v2, v3, i1, i2, i3, Color(255, 255, 255, 255));
    //    } else if (face.size() == 4) {
    //      v4 = vertices[face.vertex(3)];
    //      trV = mtr * Vec4f(v4.x(), v4.y(), v4.z(), 1);
    //      v4 = Vec3f(trV.x() / trV.w(), trV.y() / trV.w(), trV.z() / trV.w());

    //      normal = normals[face.normal(3)];
    //      normal.normalize();
    //      i4 = Vec3f::dotProduct(normal, light);

    //      _drawer->drawQuadrangle(v1, v2, v3, v4, i1, i2, i3, i4,
    //                              Color(255, 255, 255, 255));
    //    }
  }
}

void DrawVisitor::visit(const ParticleSystem& ps) {
  //  qDebug() << "DrawVisitor::visit(const ParticleSystem& model)";
  const Mtr4x4f modelView = Mtr4x4f::lookAt(_camera->eye(), Vec3f(_camera->center()), _camera->up());

  const int width = _drawer->width(), height = _drawer->height();
  const Mtr4x4f viewport = Mtr4x4f::viewport(0, 0, width, height);

  const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
                                         0, -1, 0, 0,  //
                                         0, 0, 1, 0,   //
                                         0, 0, 0, 1);
  const Mtr4x4f mtr = viewport * flipVertically * modelView;

  Vec4f tvec;
  Vec3f vel;

  for (auto& p : ps._particles) {
    Vec3f point = Vec3f(p.pos.x(), p.pos.y(), p.pos.z());

    point.x() = point.x() / 30.0;
    point.y() = point.y() / 30.0;
    point.z() = point.z() / 30.0;

    tvec = mtr * Vec4f(point.x(), point.y(), point.z(), 1);
    point = Vec3f(tvec[0] / tvec[3], tvec[1] / tvec[3], tvec[2] / tvec[3]);

    Vec3f point2;
    vel = p.vel_eval;
    if (vel.length() != 0) {
      vel.normalize();
      //      vel *= 3;
      point2 = Vec3f(p.pos.x() + vel.x(), p.pos.y() + vel.y(), p.pos.z() + vel.z());

      point2.x() = point2.x() / 30.0;
      point2.y() = point2.y() / 30.0;
      point2.z() = point2.z() / 30.0;

      tvec = mtr * Vec4f(point2.x(), point2.y(), point2.z(), 1);
      point2 = Vec3f(tvec[0] / tvec[3], tvec[1] / tvec[3], tvec[2] / tvec[3]);
    } else {
      point2 = point;
    }

    Color color = Color(255 * RED(p.clr), 255 * GRN(p.clr), 255 * BLUE(p.clr));

    if (_particleMode == SPHERE) {
      _drawer->drawSphere(point, color, Vec3f());
    } else {
      _drawer->drawVector(point, point2, color);
    }
  }
}

//// matrices
// const Mtr4x4f projection = Mtr4x4f(1, 0, 0, 0,  //
//                                   0, 1, 0, 0,  //
//                                   0, 0, 1, 0,  //
//                                   0, 0, -1.0f / (eye - center).length(), 1);
// const Mtr4x4f modelView = Mtr4x4f::lookAt(eye, center, Vec3f(0, 1, 0));
// const int width = _drawer->width(), height = _drawer->height();
// const Mtr4x4f viewport =
// Mtr4x4f::viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
// const Mtr4x4f flipVertically = Mtr4x4f(1, 0, 0, 0,   //
//                                       0, -1, 0, 0,  //
//                                       0, 0, 1, 0,   //
//                                       0, 0, 0, 1);
// const Mtr4x4f mtr = viewport * flipVertically * projection * modelView;

////  std::cout << "projection: " << std::endl << projection;
////  std::cout << "modelView: " << std::endl << modelView;
////  std::cout << "viewport: " << std::endl << viewport;
////  std::cout << "Mtr: " << std::endl << mtr;
