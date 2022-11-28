#ifndef VECTOR_ITERATOR_HPP
#define VECTOR_ITERATOR_HPP

#include "vector_iterator.h"

template <typename T>
VectorIterator<T>::VectorIterator(const VectorIterator &vectorIterator)
    : BaseIterator<T>(vectorIterator._ptr)
{
}

template<typename T>
T &VectorIterator<T>::operator*()
{
    return *this->_ptr;
}


template<typename T>
const T &VectorIterator<T>::operator*() const
{
    return *this->_ptr;
}

template<typename T>
T *VectorIterator<T>::operator->()
{
    return this->_ptr;
}

template<typename T>
const T *VectorIterator<T>::operator->() const
{
    return this->_ptr;
}

// private
template <typename T>
VectorIterator<T>::VectorIterator(T *ptr) : BaseIterator<T>(ptr)
{
}

#endif // VECTOR_ITERATOR_HPP
