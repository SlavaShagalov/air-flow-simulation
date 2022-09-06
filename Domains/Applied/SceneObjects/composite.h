#ifndef COMPOSITE_H
#define COMPOSITE_H

#include "base_object.h"

#include <Domains/Applied/Visitors/base_draw_visitor.h>
#include <Domains/Applied/Visitors/base_transform_visitor.h>

class Composite : public BaseObject {
  using Iterator = std::vector<std::shared_ptr<BaseObject>>::iterator;
  using ConstIterator =
  std::vector<std::shared_ptr<BaseObject>>::const_iterator;

public:
  Composite() = default;
  ~Composite() = default;

  explicit Composite(const std::shared_ptr<BaseObject>& component);
  explicit Composite(const std::vector<std::shared_ptr<BaseObject>>& vector);

  bool add(const std::shared_ptr<BaseObject>& component);
  bool remove(const Iterator& iterator);

  Iterator begin() { return _objects.begin(); }
  Iterator end() { return _objects.end(); }
  ConstIterator begin() const { return _objects.begin(); }
  ConstIterator end() const { return _objects.end(); }
  ConstIterator cbegin() const { return _objects.cbegin(); }
  ConstIterator cend() const { return _objects.cend(); }

  virtual void accept(BaseTransformVisitor& visitor) override {
    for (auto& elem : _objects)
      elem->accept(visitor);
  }
  virtual void accept(BaseDrawVisitor& visitor) override {
    //    for (auto& elem : _objects)
    //      elem->accept(visitor);

    std::vector<std::shared_ptr<BaseObject>>::reverse_iterator it1, it2;
    for (it1 = _objects.rbegin(), it2 = _objects.rend(); it1 != it2; ++it1) {
      (*it1)->accept(visitor);
    }
  }

  bool isVisible() const override { return false; }
  Vec3f center() const override { return _center; }
  Vec3f& center() override { return _center; }
  void updateCenter();

private:
  std::vector<std::shared_ptr<BaseObject>> _objects;
};

#endif  // COMPOSITE_H
