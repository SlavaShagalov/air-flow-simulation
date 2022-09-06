#ifndef BASE_VECTOR_H
#define BASE_VECTOR_H

#include <cstddef>
#include <initializer_list>
#include <exception>

#include <Domains/Applied/Containers/Iterator/vector_const_iterator.h>
#include <Domains/Applied/Containers/Iterator/vector_iterator.h>

template <typename T>
class BaseVector {
  const size_t BASE_SIZE = 10;

public:
  typedef VectorIterator<T> iterator;
  typedef VectorConstIterator<T> const_iterator;

  // constructors
  BaseVector();
  explicit BaseVector(const BaseVector<T>& vectorBase);
  BaseVector(BaseVector<T>&& vectorBase) noexcept;

  explicit BaseVector(size_t size);
  BaseVector(size_t size, const T& value);
  BaseVector(std::initializer_list<T> initList);

  template <typename IterT>
  BaseVector(IterT first, IterT end);
  template <typename IterT>
  BaseVector(IterT first, size_t size);

  virtual ~BaseVector();

  // operators
  BaseVector& operator=(const BaseVector& vectorBase);
  BaseVector& operator=(BaseVector&& vectorBase);
  BaseVector& operator=(std::initializer_list<T> initList);

  T& at(size_t index);
  const T& at(size_t index) const;
  T& operator[](size_t index);
  const T& operator[](size_t index) const;

  // base operations
  bool empty() const noexcept;
  size_t size() const noexcept;
  size_t capacity() const noexcept;
  void clear();
  void reallocate();

protected:
  T* _data;
  size_t _size;
  size_t _allocatedSize;
};
#endif  // BASE_VECTOR_H
