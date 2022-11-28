#ifndef BASE_VECTOR_HPP
#define BASE_VECTOR_HPP

#include "base_vector.h"

#include <QDebug>

// constructors
template <typename T>
BaseVector<T>::BaseVector() : _size(0), _allocatedSize(BASE_SIZE) {
  //    try {
  _data = new T[_allocatedSize];

  //    } catch (std::bad_alloc &ex)
  //    {
  //        throw BadMemoryAllocationException();
  //    }
}

template <typename T>
BaseVector<T>::BaseVector(const BaseVector<T>& vectorBase)
    : BaseVector(vectorBase._size) {
  std::copy(vectorBase._data, vectorBase._data + vectorBase._size, _data);
}

template <typename T>
BaseVector<T>::BaseVector(BaseVector<T>&& vectorBase) noexcept
    : _size(vectorBase._size)
    , _allocatedSize(vectorBase._allocatedSize)
    , _data(vectorBase._data) {
  vectorBase._size = 0;
  vectorBase._allocatedSize = 0;
  vectorBase._data = nullptr;
}

template <typename T>
BaseVector<T>::BaseVector(size_t size) : _size(size), _allocatedSize(size) {
  //  try {
  _data = new T[_allocatedSize];

  //    } catch (std::bad_alloc& ex) {
  //    throw BadMemoryAllocationException();
  //  }
}

template <typename T>
BaseVector<T>::BaseVector(size_t size, const T& value) : BaseVector(size) {
  for (size_t i = 0; i < _size; i++)
    (*this)[i] = value;
}

template <typename T>
BaseVector<T>::BaseVector(std::initializer_list<T> initList)
    : BaseVector(initList.size()) {
  //  qDebug() << "BASEVectorwww" << *initList.begin() << " "
  //           << *(initList.begin() + 1);

  std::copy(initList.begin(), initList.end(), _data);
}

template <typename T>
template <typename IterT>
BaseVector<T>::BaseVector(IterT begin, IterT end) : BaseVector(end - begin) {
  std::copy(begin, end, _data);
}

template <typename T>
template <typename IterT>
BaseVector<T>::BaseVector(IterT first, size_t size) : BaseVector(size) {
  std::copy(first, first + size, _data);
}

template <typename T>
BaseVector<T>::~BaseVector() {
  delete[] _data;
}

// operators
template <typename T>
T& BaseVector<T>::at(size_t index) {
  //  if (index >= _size)
  //    throw OutOfRangeException();

  return (*this)[index];
}

template <typename T>
const T& BaseVector<T>::at(size_t index) const {
  //  if (index >= _size)
  //    throw OutOfRangeException();

  return (*this)[index];
}

template <typename T>
T& BaseVector<T>::operator[](size_t index) {
  //  if (index >= _size)
  //    throw OutOfRangeException();

  return _data[index];
}

template <typename T>
const T& BaseVector<T>::operator[](size_t index) const {
  //  if (index >= _size)
  //    throw OutOfRangeException();

  return _data[index];
}

template <class T>
BaseVector<T>& BaseVector<T>::operator=(const BaseVector<T>& other) {
  if (this != &other) {
    this->_size = other._size;
    this->_allocatedSize = other._allocatedSize;

    //        try {
    delete[] this->_data;
    this->_data = new T[this->_allocatedSize];

    //        } catch (std::bad_alloc& ex) {
    //            throw bad_memory_allocation_exception("vector_base::(bad
    //            memory allocation)!");
    //        }

    std::copy(other._data, other._data + other._size, this->_data);
  }

  return *this;
}

template <class T>
BaseVector<T>& BaseVector<T>::operator=(BaseVector<T>&& other) {
  if (this != &other) {
    this->_size = other._size;
    this->_allocatedSize = other._allocatedSize;

    delete[] _data;
    _data = std::move(other._data);

    other._size = 0;
    other._allocatedSize = 0;
    other._data = nullptr;
  }

  return *this;
}

// base operations
template <typename T>
bool BaseVector<T>::empty() const noexcept {
  return _size == 0;
}

template <typename T>
size_t BaseVector<T>::size() const noexcept {
  return _size;
}

template <typename T>
size_t BaseVector<T>::capacity() const noexcept {
  return _allocatedSize;
}

template <typename T>
void BaseVector<T>::clear() {
  for (size_t i = 0; i < _size; ++i)
    (*this)[i].~T();

  _size = 0;
}

template <class T>
inline void BaseVector<T>::reallocate() {
  //    try {
  T* temp_buffer = new T[this->_allocatedSize];
  std::copy(this->_data, this->_data + this->_size, temp_buffer);
  delete[] this->_data;
  this->_data = temp_buffer;

  //    } catch (std::bad_alloc& ex) {
  //        throw bad_memory_allocation_exception("vector_base::(bad memory
  //        allocation)!");
  //    }
}
#endif  // BASE_VECTOR_HPP
