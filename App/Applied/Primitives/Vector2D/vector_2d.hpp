#ifndef VECTOR_2D_HPP
#define VECTOR_2D_HPP

#include <cassert>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include <App/Applied/Primitives/Point2D/point_2d.hpp>

template <typename T> class Vector2D;

using Vector2D_I = Vector2D<int>;
using Vector2D_F = Vector2D<float>;
using Vector2D_D = Vector2D<double>;

template <typename T> class Vector2D {
  static_assert(std::is_arithmetic<T>::value, "Type must be arithmetic.");

private:
  T _coords[2];

public:
  // constructors
  constexpr inline Vector2D() : _coords{0, 0} {}
  Vector2D(const Vector2D &other);
  Vector2D(Vector2D &&other) noexcept;

  explicit constexpr inline Vector2D(const Point2D<T> &point)
      : _coords{point.x(), point.y()} {}
  constexpr Vector2D(const T &x, const T &y) : _coords{x, y} {}
  constexpr Vector2D(std::initializer_list<T> initList)
      : _coords{*initList.begin(), *(initList.begin() + 1)} {}

  ~Vector2D() = default;

  // operators
  Vector2D &operator=(const Vector2D<T> &other);
  Vector2D &operator=(Vector2D<T> &&other) noexcept;

  T &operator[](int i);
  T operator[](int i) const;

  // getters
  T length() const;
  constexpr inline T x() const { return _coords[0]; }
  constexpr inline T y() const { return _coords[1]; }

  // setters
  inline void setX(const T &x) { _coords[0] = x; }
  inline void setY(const T &y) { _coords[1] = y; }

  // methods
  void normalize();

  // static methods
  static T dotProduct(const Vector2D &vec1, const Vector2D &vec2);
  static T cosAngle(const Vector2D &vec1, const Vector2D &vec2);

  // friend functions
  template <typename U>
  friend std::ostream &operator<<(std::ostream &os, const Vector2D<U> &vector);
};

// constructors
template <typename T> Vector2D<T>::Vector2D(const Vector2D<T> &other) {
  _coords = other._coords;
}

template <typename T> Vector2D<T>::Vector2D(Vector2D &&other) noexcept {
  for (size_t i = 0; i < 2; i++)
    _coords[i] = other._coords[i];
}

// operators
template <typename T>
Vector2D<T> &Vector2D<T>::operator=(const Vector2D<T> &other) {
  if (this != &other) {
    _coords = other._coords;
  }

  return *this;
}

template <typename T>
Vector2D<T> &Vector2D<T>::operator=(Vector2D<T> &&other) noexcept {
  if (this != &other) {
    for (size_t i = 0; i < 2; i++)
      _coords[i] = other._coords[i];
  }

  return *this;
}

template <typename T> inline T &Vector2D<T>::operator[](int i) {
  assert(-1 < i && i < 2);
  return _coords[i];
}

template <typename T> inline T Vector2D<T>::operator[](int i) const {
  assert(-1 < i && i < 2);
  return _coords[i];
}

// getters
template <typename T> T Vector2D<T>::length() const {
  const T x = _coords[0];
  const T y = _coords[1];

  return sqrt(x * x + y * y);
}

// methods
template <typename T> void Vector2D<T>::normalize() {
  const T norm = 1 / length();

  _coords[0] *= norm;
  _coords[1] *= norm;
}

// static methods
template <typename T>
T Vector2D<T>::dotProduct(const Vector2D &vec1, const Vector2D &vec2) {
  return vec1.x() * vec2.x() + vec1.y() * vec2.y();
}

template <typename T>
T Vector2D<T>::cosAngle(const Vector2D &vec1, const Vector2D &vec2) {
  return dotProduct(vec1, vec2) / (vec1.length() * vec2.length());
}

// friend functions
template <typename U>
std::ostream &operator<<(std::ostream &os, const Vector2D<U> &vector) {
  os << "[" << vector._coords[0] << "; " << vector._coords[1] << "]"
     << std::endl;
  return os;
}
#endif // VECTOR_2D_HPP
