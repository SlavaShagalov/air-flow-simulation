#ifndef VECTOR_ITERATOR_H
#define VECTOR_ITERATOR_H

template <typename T> class BaseVector;

#include <memory>

#include "base_iterator.hpp"

template <typename T>
class VectorIterator : public BaseIterator<T>
{
    friend class BaseVector<T>;

public:
    VectorIterator(const VectorIterator &vectorIterator);

    T& operator*();
    const T &operator*() const;

    T* operator->();
    const T *operator->() const;

private:
    VectorIterator(T *ptr);
};
#endif // VECTOR_ITERATOR_H
