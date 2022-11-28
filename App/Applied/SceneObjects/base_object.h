#ifndef BASE_OBJECT_H
#define BASE_OBJECT_H

#include <vector>
#include <memory>
#include <QDebug>

#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>

class BaseTransformVisitor;
class BaseDrawVisitor;

class BaseObject {
public:
  using Iterator = std::vector<std::shared_ptr<BaseObject>>::iterator;
  using ConstIterator =
  std::vector<std::shared_ptr<BaseObject>>::const_iterator;

  BaseObject() = default;
  BaseObject(const BaseObject& other) = default;
  BaseObject(BaseObject&& other) = default;

  explicit BaseObject(const Vec3f& center) : _center(center) {}
  virtual ~BaseObject() = default;

  virtual void accept(BaseTransformVisitor& visitor) = 0;
  virtual void accept(BaseDrawVisitor& visitor) = 0;

  virtual bool isVisible() const = 0;
  virtual Vec3f& center() = 0;
  virtual Vec3f center() const = 0;

protected:
  Vec3f _center;
};

#endif  // BASE_OBJECT_H
