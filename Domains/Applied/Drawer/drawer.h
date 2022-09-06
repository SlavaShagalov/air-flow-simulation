#ifndef DRAWER_H
#define DRAWER_H

#include <QPolygon>
#include <memory>

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>

#include <Domains/Applied/Color/color.h>

class BaseDrawer {
public:
  BaseDrawer() = default;

  virtual ~BaseDrawer() = default;

  virtual void drawPolygon(const QPolygon& polygon,
                           const Color& color = Color()) = 0;

  // draw quadrangle
  virtual void drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              const Color& color = Color()) = 0;
  virtual void drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              float& i1, float& i2, float& i3, float& i4,
                              const Color& color = Color()) = 0;

  // draw triangle
  virtual void drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3,
                            const Color& color = Color()) = 0;
  virtual void drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, float& i1,
                            float& i2, float& i3,
                            const Color& color = Color()) = 0;

  // draw line
  virtual void drawLine(const Vec3f& v1, const Vec3f& v2,
                        const Color& color = Color()) = 0;
  virtual void drawLineWidth(const Vec3f& v1, const Vec3f& v2,
                             const Color& color = Color(),
                             const int width = 1) = 0;

  // draw vector
  virtual void drawVector(const Vec3f& v1, const Vec3f& v2,
                          const Color& color = Color()) = 0;

  // draw point
  virtual void drawSphere(const Vec3f& point, const Color& color,
                            const Vec3f& vel) = 0;

  virtual void clear() = 0;
  virtual void update() = 0;

  virtual int width() const = 0;
  virtual int height() const = 0;
};

class DrawerFactory {
public:
  virtual std::unique_ptr<BaseDrawer> createDrawer() = 0;
};

#endif  // DRAWER_H
