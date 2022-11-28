#ifndef VECTOR_HPP
#define VECTOR_HPP

#include "vector.h"

#include <iostream>

// constructors
template <typename T>
Vector<T>::Vector() : BaseVector<T>() {}

template <typename T>
Vector<T>::Vector(const Vector<T>& other) : BaseVector<T>(other) {}

template <typename T>
Vector<T>::Vector(Vector<T>&& other) noexcept
    : BaseVector<T>(std::move(other)) {}

template <typename T>
Vector<T>::Vector(size_t size) : BaseVector<T>(size) {}

template <typename T>
Vector<T>::Vector(size_t size, const T& value) : BaseVector<T>(size, value) {}

template <typename T>
Vector<T>::Vector(std::initializer_list<T> initList) : BaseVector<T>(initList) {
  //  qDebug() << "Vectorwww" << *initList.begin() << " "
  //           << *(initList.begin() + 1);
}

template <typename T>
template <typename IterT>
Vector<T>::Vector(IterT begin, IterT end) : BaseVector<T>(begin, end) {}

template <typename T>
template <typename IterT>
Vector<T>::Vector(IterT begin, size_t size) : BaseVector<T>(begin, size) {}

template <class T>
void Vector<T>::push_back(const T& value) {
  if (this->size() == this->capacity()) {
    this->_allocatedSize <<= RESIZE_COEFF;
    this->reallocate();
  }

  (*this)[this->_allocatedSize++] = value;
}

template <class T>
Vector<T>& Vector<T>::operator=(const Vector<T>& other) {
  BaseVector<T>::operator=(other);
  return *this;
}

template <class T>
Vector<T>& Vector<T>::operator=(Vector<T>&& other) {
  BaseVector<T>::operator=(other);
  return *this;
}

// operators
// template <typename T>
// Vector<T>& Vector<T>::operator=(std::initializer_list<T> initList) {}
#endif  // VECTOR_HPP
