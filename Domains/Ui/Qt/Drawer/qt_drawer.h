#ifndef QT_DRAWER_H
#define QT_DRAWER_H

#include <memory>

#include <QColor>
#include <QDebug>
#include <QGraphicsScene>
#include <QPen>

#include <Domains/Applied/Drawer/drawer.h>

#include <Domains/Applied/Color/color.h>

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>

class QtDrawer : public BaseDrawer {
public:
  QtDrawer() = delete;

  explicit QtDrawer(std::shared_ptr<QGraphicsScene>& scene);

  QtDrawer(const QtDrawer& drawer);

  virtual void drawPolygon(const QPolygon& polygon,
                           const Color& color = Color()) override;

  // draw quadrangle
  virtual void drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              const Color& color = Color()) override;
  virtual void drawQuadrangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, Vec3f& v4,
                              float& i1, float& i2, float& i3, float& i4,
                              const Color& color = Color()) override;

  // draw triangle
  virtual void drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3,
                            const Color& color) override;
  virtual void drawTriangle(Vec3f& v1, Vec3f& v2, Vec3f& v3, float& i1,
                            float& i2, float& i3, const Color& color) override;

  // draw line
  virtual void drawLine(const Vec3f& v1, const Vec3f& v2,
                        const Color& color = Color()) override;
  virtual void drawLineWidth(const Vec3f& v1, const Vec3f& v2,
                             const Color& color = Color(),
                             const int width = 1) override;

  // draw vector
  virtual void drawVector(const Vec3f& v1, const Vec3f& v2,
                          const Color& color = Color()) override;

  // draw point
  virtual void drawSphere(const Vec3f& point, const Color& color,
                            const Vec3f& vel) override;

  // scene methods
  virtual void clear() override;
  virtual void update() override;
  virtual int width() const override { return _width; }
  virtual int height() const override { return _height; }

private:
  void _drawPoint(const int x, const int y, const int z, const QColor& color);

private:
  std::shared_ptr<QGraphicsScene> _qScene;
  std::unique_ptr<QImage> _qImage;
  //  std::vector<int> _zBuffer;
  std::vector<float> _zBuffer;

  const int _width = 0;
  const int _height = 0;
  const int _width_2 = 0;
  const int _height_2 = 0;
};

#endif  // QT_DRAWER_H
