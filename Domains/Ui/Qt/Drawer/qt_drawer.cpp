#include "qt_drawer.h"

#include <QException>
#include <QMatrix4x4>
#include <QPoint>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

QtDrawer::QtDrawer(std::shared_ptr<QGraphicsScene>& scene)
    : _qScene(scene)
    , _zBuffer(scene->width() * scene->height())
    , _width(scene->width())
    , _height(scene->height())
    , _width_2(scene->width() / 2)
    , _height_2(scene->height() / 2) {
  qDebug() << "QtDrawer created: width =" << _width << "height =" << _height;
  qDebug() << "Z-Buffer: size = " << _zBuffer.size();

  _qImage = std::make_unique<QImage>(scene->width(), scene->height(),
                                     QImage::Format_ARGB32);
  _qImage->fill(Qt::transparent);

  // fill z-buffer
  for (size_t i = 0; i < _zBuffer.size(); i++)
    _zBuffer[i] = std::numeric_limits<float>::lowest();
}

QtDrawer::QtDrawer(const QtDrawer& drawer) : _qScene(drawer._qScene) {}

void QtDrawer::drawPolygon(const QPolygon& polygon, const Color& color) {
  const QColor qColor(color.red(), color.green(), color.blue(), color.alpha());

  //  _qScene->addPolygon(polygon, QPen(qColor), QBrush(qColor));
  _qScene->addPolygon(polygon, QPen(qColor));
}

// draw quadrangle
void QtDrawer::drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              const Color& color) {}

void QtDrawer::drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              float& i1, float& i2, float& i3, float& i4,
                              const Color& color) {}

//// simple draw
// void QtDrawer::drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3,
//                            const Color& color = Color()) {
//  // convert input data
//  int x1 = v1.x(), x2 = v2.x(), x3 = v3.x();
//  int y1 = v1.y(), y2 = v2.y(), y3 = v3.y();
//  int z1 = v1.z(), z2 = v2.z(), z3 = v3.z();

//  // sort points
//  if (y2 < y1) {
//    std::swap(x1, x2);
//    std::swap(y1, y2);
//    std::swap(z1, z2);
//  }
//  if (y3 < y1) {
//    std::swap(x1, x3);
//    std::swap(y1, y3);
//    std::swap(z1, z3);
//  }
//  if (y2 > y3) {
//    std::swap(x2, x3);
//    std::swap(y2, y3);
//    std::swap(z2, z3);
//  }

//  if (y1 == y3)
//    return;

//  // find steps
//  const int dy13 = y3 - y1, dy12 = y2 - y1, dy23 = y3 - y2;
//  const int dx13 = x3 - x1, dx12 = x2 - x1, dx23 = x3 - x2;

//  float xStep13, xStep12, xStep23;
//  if (y3 != y1)
//    xStep13 = (float)dx13 / (float)dy13;
//  else
//    xStep13 = 0;
//  if (y2 != y1)
//    xStep12 = (float)dx12 / (float)dy12;
//  else
//    xStep12 = 0;
//  if (y3 != y2)
//    xStep23 = (float)dx23 / (float)dy23;
//  else
//    xStep23 = 0;

//  float xBeg = x1, xEnd = x1, zBeg, zEnd, z;
//  float _xStep13 = xStep13;

//  bool flag = false;
//  if (xStep13 > xStep12) {
//    std::swap(xStep13, xStep12);
//    flag = true;
//  }
//  float alpha, beta, phi;

//  // fill bottom triangle
//  for (int y = y1; y < y2; y++) {
//    alpha = (float)(y - y1) / (float)(y3 - y1);
//    beta = (float)(y - y1) / (float)(y2 - y1);

//    zBeg = z1 + alpha * (z3 - z1);
//    zEnd = z1 + beta * (z2 - z1);

//    if (flag) {
//      std::swap(zBeg, zEnd);
//    }

//    // fill horizontal line
//    for (int x = xBeg; x <= xEnd; x++) {
//      if (x < 0 || x >= _width || y < 0 || y >= _height)
//        continue;

//      phi = xEnd == xBeg ? 1 : (x - xBeg) / (xEnd - xBeg);

//      z = zBeg + phi * (zEnd - zBeg);

//      int idx = x + y * _width;
//      if (_zBuffer[idx] < z) {
//        _zBuffer[idx] = z;

//        _qImage->setPixelColor(
//        x, y, QColor(color.red(), color.green(), color.blue(),
//        color.alpha()));
//      }
//    }

//    xBeg += xStep13;
//    xEnd += xStep12;
//  }

//  flag = false;

//  if (y1 == y2) {
//    if (x2 > x1) {
//      xBeg = x1;
//      xEnd = x2;
//    } else {
//      xBeg = x2;
//      xEnd = x1;
//    }
//  }

//  if (_xStep13 < xStep23) {
//    flag = true;
//    std::swap(_xStep13, xStep23);
//  }

//  // fill top triangle
//  for (int y = y2; y <= y3; y++) {
//    alpha = (double)(y - y1) / (double)(y3 - y1);
//    beta = (double)(y - y2) / (double)(y3 - y2);

//    zBeg = z1 + alpha * (z3 - z1);
//    zEnd = z2 + beta * (z3 - z2);

//    if (flag) {
//      std::swap(zBeg, zEnd);
//    }

//    // fill horizontal line
//    for (int x = xBeg; x <= xEnd; x++) {
//      if (x < 0 || x >= _width || y < 0 || y >= _height)
//        continue;

//      phi = xEnd == xBeg ? 1 : (x - xBeg) / (xEnd - xBeg);

//      z = zBeg + phi * (zEnd - zBeg);

//      int idx = x + y * _width;
//      if (_zBuffer[idx] < z) {
//        _zBuffer[idx] = z;

//        _qImage->setPixelColor(
//        x, y, QColor(color.red(), color.green(), color.blue(),
//        color.alpha()));
//      }
//    }

//    xBeg += _xStep13;
//    xEnd += xStep23;
//  }
//}

void QtDrawer::drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3,
                            const Color& color = Color()) {
  //  qDebug() << " drawTriangle()";

  // convert input data
  int x1 = v1.x(), x2 = v2.x(), x3 = v3.x();
  int y1 = v1.y(), y2 = v2.y(), y3 = v3.y();
  int z1 = v1.z(), z2 = v2.z(), z3 = v3.z();

  // sort vertices
  if (y2 < y1) {
    std::swap(x1, x2);
    std::swap(y1, y2);
    std::swap(z1, z2);
  }
  if (y3 < y1) {
    std::swap(x1, x3);
    std::swap(y1, y3);
    std::swap(z1, z3);
  }
  if (y3 < y2) {
    std::swap(x2, x3);
    std::swap(y2, y3);
    std::swap(z2, z3);
  }

  if (y1 == y3)
    return;

  // find diffs
  const int dy13 = y3 - y1, dy12 = y2 - y1, dy23 = y3 - y2;
  const int dx13 = x3 - x1, dx12 = x2 - x1, dx23 = x3 - x2;

  float xStep13, xStep12, xStep23;

  xStep13 = (float)dx13 / (float)dy13;
  if (y2 != y1)
    xStep12 = (float)dx12 / (float)dy12;
  else
    xStep12 = 0;
  if (y3 != y2)
    xStep23 = (float)dx23 / (float)dy23;
  else
    xStep23 = 0;

  float xBeg = x1, xEnd = x1, zBeg, zEnd, z;
  float _xStep13 = xStep13;  // save

  bool flag;
  if (xStep13 > xStep12) {
    std::swap(xStep13, xStep12);
    flag = true;
  } else {
    flag = false;
  }

  float k13, k12, k23, k;

  // fill bottom triangle
  for (int y = y1; y < y2; y++) {
    k13 = (float)(y - y1) / (float)(y3 - y1);
    k12 = (float)(y - y1) / (float)(y2 - y1);

    zBeg = z1 + k13 * (z3 - z1);
    zEnd = z1 + k12 * (z2 - z1);

    if (flag) {
      std::swap(zBeg, zEnd);
    }

    // fill horizontal line
    for (int x = xBeg; x <= xEnd; x++) {
      if (x < 0 || x >= _width || y < 0 || y >= _height)
        continue;

      if (xEnd == xBeg) {
        z = zBeg;
      } else {
        k = (float)(x - xBeg) / (float)(xEnd - xBeg);
        z = zBeg + k * (zEnd - zBeg);
      }

      int idx = x + y * _width;
      if (_zBuffer[idx] < z) {
        _zBuffer[idx] = z;

        _qImage->setPixelColor(
        x, y, QColor(color.red(), color.green(), color.blue(), color.alpha()));
      }
    }

    xBeg += xStep13;
    xEnd += xStep12;
  }

  if (y1 == y2) {
    if (x2 > x1) {
      xBeg = x1;
      xEnd = x2;
    } else {
      xBeg = x2;
      xEnd = x1;
    }
  }

  if (_xStep13 < xStep23) {
    flag = true;
    std::swap(_xStep13, xStep23);
  } else {
    flag = false;
  }

  // fill top triangle
  for (int y = y2; y <= y3; y++) {
    k13 = (float)(y - y1) / (float)(y3 - y1);
    k23 = (float)(y - y2) / (float)(y3 - y2);  // TODO: div 0

    zBeg = z1 + k13 * (z3 - z1);
    zEnd = z2 + k23 * (z3 - z2);

    if (flag) {
      std::swap(zBeg, zEnd);
    }

    // fill horizontal line
    for (int x = xBeg; x <= xEnd; x++) {
      if (x < 0 || x >= _width || y < 0 || y >= _height)
        continue;

      if (xEnd == xBeg) {
        z = zBeg;
      } else {
        k = (float)(x - xBeg) / (float)(xEnd - xBeg);
        z = zBeg + k * (zEnd - zBeg);
      }

      int idx = x + y * _width;
      if (_zBuffer[idx] < z) {
        _zBuffer[idx] = z;

        _qImage->setPixelColor(
        x, y, QColor(color.red(), color.green(), color.blue(), color.alpha()));
      }
    }

    xBeg += _xStep13;
    xEnd += xStep23;
  }
}

void QtDrawer::drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, float& _i1,
                            float& _i2, float& _i3,
                            const Color& color = Color()) {
  // convert input data
  int x1 = v1.x(), x2 = v2.x(), x3 = v3.x();
  int y1 = v1.y(), y2 = v2.y(), y3 = v3.y();
  int z1 = v1.z(), z2 = v2.z(), z3 = v3.z();
  float i1 = _i1, i2 = _i2, i3 = _i3;

  // sort points
  if (y2 < y1) {
    std::swap(x1, x2);
    std::swap(y1, y2);
    std::swap(z1, z2);
    std::swap(i1, i2);
  }
  if (y3 < y1) {
    std::swap(x1, x3);
    std::swap(y1, y3);
    std::swap(z1, z3);
    std::swap(i1, i3);
  }
  if (y2 > y3) {
    std::swap(x2, x3);
    std::swap(y2, y3);
    std::swap(z2, z3);
    std::swap(i2, i3);
  }

  if (y1 == y3)
    return;

  // find steps
  const int dy13 = y3 - y1, dy12 = y2 - y1, dy23 = y3 - y2;
  const int dx13 = x3 - x1, dx12 = x2 - x1, dx23 = x3 - x2;

  double xStep13, xStep12, xStep23;
  if (y3 != y1)
    xStep13 = (double)dx13 / (double)dy13;
  else
    xStep13 = 0;
  if (y2 != y1)
    xStep12 = (double)dx12 / (double)dy12;
  else
    xStep12 = 0;
  if (y3 != y2)
    xStep23 = (double)dx23 / (double)dy23;
  else
    xStep23 = 0;

  double xBeg = x1, xEnd = x1, zBeg, zEnd, z, iBeg, iEnd, i;
  double _xStep13 = xStep13;

  bool flag = false;
  if (xStep13 > xStep12) {
    std::swap(xStep13, xStep12);
    flag = true;
  }
  double alpha, beta, phi;

  // fill bottom triangle
  for (int y = y1; y < y2; y++) {
    alpha = (double)(y - y1) / (double)(y3 - y1);
    beta = (double)(y - y1) / (double)(y2 - y1);

    zBeg = z1 + alpha * (z3 - z1);
    zEnd = z1 + beta * (z2 - z1);
    iBeg = i1 + alpha * (i3 - i1);
    iEnd = i1 + beta * (i2 - i1);

    if (flag) {
      std::swap(zBeg, zEnd);
      std::swap(iBeg, iEnd);
    }

    // fill horizontal line
    for (int x = xBeg; x <= xEnd; x++) {
      if (x < 0 || x >= _width || y < 0 || y >= _height)
        continue;

      phi = xEnd == xBeg ? 1 : (x - xBeg) / (xEnd - xBeg);

      z = zBeg + phi * (zEnd - zBeg);
      i = iBeg + phi * (iEnd - iBeg);

      int idx = x + y * _width;
      if (_zBuffer[idx] < z) {
        _zBuffer[idx] = z;

        if (i > 1)
          i = 1.0;
        if (i < 0)
          i = 0.0;
        _qImage->setPixelColor(x, y,
                               QColor(color.red() * i, color.green() * i,
                                      color.blue() * i, color.alpha()));
      }
    }

    xBeg += xStep13;
    xEnd += xStep12;
  }

  flag = false;

  if (y1 == y2) {
    if (x2 > x1) {
      xBeg = x1;
      xEnd = x2;
    } else {
      xBeg = x2;
      xEnd = x1;
    }
  }

  if (_xStep13 < xStep23) {
    flag = true;
    std::swap(_xStep13, xStep23);
  }

  // fill top triangle
  for (int y = y2; y <= y3; y++) {
    alpha = (double)(y - y1) / (double)(y3 - y1);
    beta = (double)(y - y2) / (double)(y3 - y2);

    zBeg = z1 + alpha * (z3 - z1);
    zEnd = z2 + beta * (z3 - z2);
    iBeg = i1 + alpha * (i3 - i1);
    iEnd = i2 + beta * (i3 - i2);

    if (flag) {
      std::swap(zBeg, zEnd);
      std::swap(iBeg, iEnd);
    }

    // fill horizontal line
    for (int x = xBeg; x <= xEnd; x++) {
      if (x < 0 || x >= _width || y < 0 || y >= _height)
        continue;

      phi = xEnd == xBeg ? 1 : (x - xBeg) / (xEnd - xBeg);

      z = zBeg + phi * (zEnd - zBeg);
      i = iBeg + phi * (iEnd - iBeg);

      int idx = x + y * _width;
      if (_zBuffer[idx] < z) {
        _zBuffer[idx] = z;

        if (i > 1)
          i = 1.0;
        if (i < 0)
          i = 0.0;
        _qImage->setPixelColor(x, y,
                               QColor(color.red() * i, color.green() * i,
                                      color.blue() * i, color.alpha()));
      }
    }

    xBeg += _xStep13;
    xEnd += xStep23;
  }
}

///// realization: use bresenham int algorithm
// void QtDrawer::drawLine(const Vec3f& v1, const Vec3f& v2, const Color& color)
// {
//  QColor qColor(color.red(), color.green(), color.blue(), color.alpha());

//  if (v1.x() == v2.x() && v1.y() == v2.y())  // TODO: opt
//  {
//    _qImage->setPixelColor(v1.x(), v1.y(), qColor);
//    return;
//  }

//  int dx = v2.x() - v1.x();
//  int dy = v2.y() - v1.y();

//  int sx = sign(dx);
//  int sy = sign(dy);

//  dx = abs(dx);
//  dy = abs(dy);

//  const bool isChanged = dy > dx;
//  if (isChanged)
//    std::swap(dx, dy);

//  const int dx_2 = dx * 2, dy_2 = dy * 2;

//  int e = dy_2 - dx;

//  int x = v1.x(), y = v1.y();

//  for (int i = 0; i <= dx; i++) {
//    _qImage->setPixelColor(x, y, qColor);

//    while (e >= 0) {
//      if (isChanged)
//        x += sx;
//      else
//        y += sy;

//      e -= dx_2;
//    }

//    if (isChanged)
//      y += sy;
//    else
//      x += sx;

//    e += dy_2;
//  }
//}

/// realization: use DDA algorithm
void QtDrawer::drawLine(const Vec3f& v1, const Vec3f& v2, const Color& color) {
  QColor qColor(color.red(), color.green(), color.blue(), color.alpha());

  const int x1 = v1.x(), y1 = v1.y(), x2 = v2.x(), y2 = v2.y();
  const float z1 = v1.z(), z2 = v2.z();

  if (x1 == x2 && y1 == y2) {
    if (!(x1 < 0 || x1 >= _width || y1 < 0 || y1 >= _height)) {
      int idx = x1 + y1 * _width;
      if (_zBuffer[idx] < std::max(z1, z2)) {
        _zBuffer[idx] = std::max(z1, z2);
        _qImage->setPixelColor(x1, y1, qColor);
      }
    }

    return;
  }

  const int dx = x2 - x1, dy = y2 - y1;
  const float dz = z2 - z1;
  int len = abs(dx);
  if (abs(dy) > len)
    len = abs(dy);

  float xStep = (float)dx / len, yStep = (float)dy / len,
        zStep = (float)dz / len;
  float x = x1, y = y1, z = z1;

  for (int i = 0; i <= len; i++, x += xStep, y += yStep, z += zStep) {
    if (x < 0 || x >= _width || y < 0 || y >= _height)
      continue;
    //    qDebug() << "(" << x << "," << y << "," << z << ")";

    int idx = round(x) + round(y) * _width;
    if (_zBuffer[idx] < z) {
      _zBuffer[idx] = z;
      _qImage->setPixelColor(round(x), round(y), qColor);
      //        qDebug() << "z if = " << _zBuffer[idx] << "and = " << z;
    }
  }
}

// void QtDrawer::drawLineWidth(const Vec3f& v1, const Vec3f& v2,
//                             const Color& color, const int width) {
//  const int x1 = v1.x(), y1 = v1.y(), x2 = v2.x(), y2 = v2.y();
//  QColor qColor(color.red(), color.green(), color.blue(), color.alpha());

//  float xinc, yinc, x, y;
//  float dx, dy, e;
//  dx = abs(x2 - x1);
//  dy = abs(y2 - y1);
//  if (x1 < x2)
//    xinc = 1;
//  else
//    xinc = -1;
//  if (y1 < y2)
//    yinc = 1;
//  else
//    yinc = -1;
//  x = x1;
//  y = y1;
//  _qImage->setPixelColor(round(x), round(y), qColor);
//  if (dx >= dy) {
//    e = (2 * dy) - dx;
//    while (x != x2) {
//      if (e < 0) {
//        e += (2 * dy);
//      } else {
//        e += (2 * (dy - dx));
//        y += yinc;
//      }
//      x += xinc;
//      _qImage->setPixelColor(round(x), round(y), qColor);
//    }
//  } else {
//    e = (2 * dx) - dy;
//    while (y != y2) {
//      if (e < 0) {
//        e += (2 * dx);
//      } else {
//        e += (2 * (dx - dy));
//        x += xinc;
//      }
//      y += yinc;
//      _qImage->setPixelColor(round(x), round(y), qColor);
//    }
//  }
//}

void QtDrawer::drawLineWidth(const Vec3f& v1, const Vec3f& v2,
                             const Color& color, const int thickness) {
  const float x1 = v1.x(), y1 = v1.y(), x2 = v2.x(), y2 = v2.y();
  const float z1 = v1.z(), z2 = v2.z();
  int i;
  float wy, wx;

  drawLine(Vec3f(x1, y1, z1), Vec3f(x2, y2, z2), color);
  Color tmp =
  Color(color.red(), color.green(), color.blue(), color.alpha() - 20);
  if ((y2 - y1) / (x2 - x1) < 1) {
    wy = (thickness - 1) * sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)) /
         (2 * fabs(x2 - x1));
    for (i = 0; i < wy; i++) {
      drawLine(Vec3f(x1, y1 - i, z1), Vec3f(x2, y2 - i, z2), tmp);
      drawLine(Vec3f(x1, y1 + i, z1), Vec3f(x2, y2 + i, z2), tmp);
      tmp.setAlpha(tmp.alpha() - 20);
    }
  } else {
    wx = (thickness - 1) * sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)) /
         (2 * fabs(y2 - y1));
    for (i = 0; i < wx; i++) {
      drawLine(Vec3f(x1 - i, y1, z1), Vec3f(x2 - i, y2, z2), tmp);
      drawLine(Vec3f(x1 + i, y1, z1), Vec3f(x2 + i, y2, z2), tmp);
      tmp.setAlpha(tmp.alpha() - 20);
    }
  }
}

void QtDrawer::_drawPoint(const int x, const int y, const int z,
                          const QColor& qColor) {
  //  qDebug() << "_drawPoint()";
  if (x < 0 || x >= _width || y < 0 || y >= _height)
    return;

  int idx = x + y * _width;
  if (_zBuffer[idx] < z) {
    _zBuffer[idx] = z;
    _qImage->setPixelColor(x, y, qColor);
    //        qDebug() << "z if = " << _zBuffer[idx] << "and = " << z;
  } else {
    //    qDebug() << "z else = " << _zBuffer[idx] << "and = " << z;
  }
}

void QtDrawer::drawSphere(const Vec3f& point, const Color& color,
                          const Vec3f& vel) {
  const QColor qColor(color.red(), color.green(), color.blue());

  //  if (point.x() - 5 < 0 || point.x() + 5 >= _width || point.y() - 5 < 0 ||
  //      point.y() + 5 >= _height)
  //    return;

  //  _qScene->addLine(point.x(), point.y(), point.x() + vel.x(),
  //                   point.y() + vel.y(), QPen(qColor));

  //  _qScene->addRect(point.x(), point.y(), 4, 4, QPen(qColor),
  //  QBrush(qColor));
}

// TODO: Optimize
void QtDrawer::drawVector(const Vec3f& v1, const Vec3f& v2,
                          const Color& color) {
  //  qDebug() << "QtDrawer::drawVector() START";
  drawLine(v1, v2, color);

  //  Vec3f v((v1.x() - v2.x()) / 3, (v1.y() - v2.y()) / 3, v2.z());
  //  const float angle = 25.0 / 180 * M_PI;

  //  float x = cos(angle) * (v.x()) - sin(angle) * (v.y()) + v2.x();
  //  float y = sin(angle) * (v.x()) + cos(angle) * (v.y()) + v2.y();

  //  //  drawLineWidth(v2, Vec3f(x, y, v.z()), color, 3);  // 1st extra line

  //  float x2 = cos(-angle) * (v.x()) - sin(-angle) * (v.y()) + v2.x();
  //  float y2 = sin(-angle) * (v.x()) + cos(-angle) * (v.y()) + v2.y();

  //  Vec3f vt1 = Vec3f(v2.x(), v2.y(), v2.z());
  //  Vec3f vt2 = Vec3f(x, y, v2.z());
  //  Vec3f vt3 = Vec3f(x2, y2, v2.z());

  //  drawTriangle(vt1, vt2, vt3, color);

  //  drawLineWidth(v1, v2, color, 5);  // main line

  //  drawLineWidth(v2, Vec3f(x, y, v.z()), color, 3);  // 2st extra line
  //  qDebug() << "QtDrawer::drawVector() END";
}

void QtDrawer::clear() {
  _qImage->fill(Qt::transparent);

  // clear z-buffer
  for (size_t i = 0; i < _zBuffer.size(); i++)
    _zBuffer[i] = std::numeric_limits<float>::lowest();

  _qScene->clear();
}

void QtDrawer::update() {
  //  qDebug() << "updateScene()";
  _qScene->clear();
  _qScene->addPixmap(QPixmap::fromImage(*_qImage));
}
