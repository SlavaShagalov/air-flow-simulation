#ifndef VECTOR_H
#define VECTOR_H

#include "base_vector.hpp"

#include <cstddef>
#include <initializer_list>
#include <exception>

template <typename T>
class Vector : public BaseVector<T> {
  const size_t RESIZE_COEFF = 2;

public:
  typedef typename BaseVector<T>::iterator iterator;
  typedef typename BaseVector<T>::const_iterator const_iterator;

  // constructors
  Vector();
  explicit Vector(const Vector<T>& vectorBase);
  Vector(Vector<T>&& vectorBase) noexcept;

  explicit Vector(size_t size);
  Vector(size_t size, const T& value);
  Vector(std::initializer_list<T> initList);

  template <typename IterT>
  Vector(IterT first, IterT end);
  template <typename IterT>
  Vector(IterT first, size_t size);

  virtual ~Vector() = default;

  void push_back(const T& value);

  // operators
  Vector& operator=(const Vector& vectorBase);
  Vector& operator=(Vector&& vectorBase);
  //  Vector& operator=(std::initializer_list<T> initializerList);
};
#endif  // VECTOR_H
