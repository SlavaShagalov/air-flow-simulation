#ifndef VECTOR_CONST_ITERATOR_HPP
#define VECTOR_CONST_ITERATOR_HPP

#include "vector_const_iterator.h"

template <typename T>
VectorConstIterator<T>::VectorConstIterator(const VectorConstIterator &other)
    : BaseIterator<T>(other._ptr)
{
}

template<typename T>
const T &VectorConstIterator<T>::operator*() const
{
    return *this->_ptr;
}

template<typename T>
const T *VectorConstIterator<T>::operator->() const
{
    return this->_ptr;
}

// private
template <typename T>
VectorConstIterator<T>::VectorConstIterator(T *ptr) : BaseIterator<T>(ptr)
{
}

#endif // VECTOR_CONST_ITERATOR_HPP
