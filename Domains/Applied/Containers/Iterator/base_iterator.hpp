#ifndef BASE_ITERATOR_HPP
#define BASE_ITERATOR_HPP

#include "base_iterator.h"

// constructors
template<typename T>
BaseIterator<T>::BaseIterator(const BaseIterator<T> &baseIterator)
{
    _ptr = baseIterator._ptr;
}

template<typename T>
BaseIterator<T>::BaseIterator(T *ptr)
{
    _ptr = ptr;
}

template <typename T>
BaseIterator<T>::~BaseIterator()
{
}

// operators
template<typename T>
BaseIterator<T> &BaseIterator<T>::operator=(const BaseIterator &baseIterator)
{
    if (this != &baseIterator)
        _ptr = baseIterator._ptr;

    return *this;
}

template<typename T>
BaseIterator<T> &BaseIterator<T>::operator++()
{
    ++_ptr;
    return *this;
}

template<typename T>
BaseIterator<T> BaseIterator<T>::operator++(int)
{
    BaseIterator<T> savedBaseIterator(*this);
    _ptr++;
    return savedBaseIterator;
}

template<typename T>
BaseIterator<T> &BaseIterator<T>::operator--()
{
    --_ptr;
    return *this;
}

template<typename T>
BaseIterator<T> BaseIterator<T>::operator--(int)
{
    BaseIterator<T> savedBaseIterator(*this);
    _ptr--;
    return savedBaseIterator;
}

template<typename T>
ptrdiff_t BaseIterator<T>::operator-(const BaseIterator<T> &baseIterator)
{
    return _ptr - baseIterator._ptr;
}

template<typename T>
bool BaseIterator<T>::operator!=(const BaseIterator<T> &baseIterator)
{
    return _ptr != baseIterator._ptr;
}

template<typename T>
bool BaseIterator<T>::operator==(const BaseIterator<T> &baseIterator)
{
    return _ptr == baseIterator._ptr;
}

#endif // BASE_ITERATOR_HPP
