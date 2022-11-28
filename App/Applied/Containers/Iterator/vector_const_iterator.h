#ifndef VECTOR_CONST_ITERATOR_H
#define VECTOR_CONST_ITERATOR_H

template <typename T> class BaseVector;

#include <memory>

#include "base_iterator.hpp"

template <typename T>
class VectorConstIterator : public BaseIterator<T>
{
    friend class BaseVector<T>;

public:
    VectorConstIterator(const VectorConstIterator &vectorIterator);

    const T &operator*() const;
    const T *operator->() const;

private:
    VectorConstIterator(T *ptr);
};
#endif // VECTOR_CONST_ITERATOR_H
