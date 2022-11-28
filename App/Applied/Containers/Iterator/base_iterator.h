#ifndef BASE_ITERATOR_H
#define BASE_ITERATOR_H

#include <memory>

template <typename T>
class BaseIterator
{
public:
    BaseIterator(const BaseIterator &baseIterator);
    virtual ~BaseIterator();

    // operators
    BaseIterator<T> &operator=(const BaseIterator &baseIterator);

    BaseIterator<T> &operator++();
    BaseIterator<T> operator++(int);
    BaseIterator<T> &operator--();
    BaseIterator<T> operator--(int);

    ptrdiff_t operator-(const BaseIterator<T> &baseIterator);

    bool operator!=(const BaseIterator<T> &baseIterator);
    bool operator==(const BaseIterator<T> &baseIterator);

protected:
    BaseIterator(T *ptr);

    std::weak_ptr<T> _ptr;
};
#endif // BASE_ITERATOR_H
