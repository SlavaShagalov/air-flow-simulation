#ifndef VECTOR_3D_HPP
#define VECTOR_3D_HPP

#include <iostream>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <utility>

template <typename T>
class Vector3D;

using Vec3i = Vector3D<int>;
using Vec3f = Vector3D<float>;
using Vec3d = Vector3D<double>;

template <typename T>
class Vector3D {
  static_assert(std::is_arithmetic<T>::value, "Type must be arithmetic.");

private:
  T _coords[3];

public:
  // constructors
  constexpr inline Vector3D() : _coords{ 0, 0, 0 } {}
  Vector3D(const Vector3D& other);
  Vector3D(Vector3D&& other) noexcept;

  explicit constexpr inline Vector3D(const T& value)
      : _coords{ value, value, value } {}
  constexpr Vector3D(const T& x, const T& y, const T& z) : _coords{ x, y, z } {}
  constexpr Vector3D(std::initializer_list<T> initList)
      : _coords{ *initList.begin(), *(initList.begin() + 1),
                 *(initList.begin() + 2) } {}

  ~Vector3D() = default;

  // operators
  Vector3D& operator=(const Vector3D<T>& other);
  Vector3D& operator=(Vector3D<T>&& other) noexcept;

  T& operator[](int i);
  T operator[](int i) const;

  Vector3D operator+(const Vector3D<T>& other) const;
  Vector3D operator-(const Vector3D<T>& other) const;
  Vector3D operator*(const T& value) const;
  Vector3D operator/(const T& value) const;

  Vector3D& operator+=(const Vector3D& other);
  Vector3D& operator+=(const T& value);

  Vector3D& operator-=(const Vector3D& other);
  Vector3D& operator-=(const T& value);

  Vector3D& operator*=(const Vector3D& other);
  Vector3D& operator*=(const T& value);

  Vector3D& operator/=(const Vector3D& other);

  bool operator==(const Vector3D<T>& other) const {
    return other._coords[0] == _coords[0] && other._coords[1] == _coords[1] &&
           other._coords[2] == _coords[2];
  }

  // getters
  T length() const;
  constexpr inline T x() const { return _coords[0]; }
  constexpr inline T y() const { return _coords[1]; }
  constexpr inline T z() const { return _coords[2]; }
  constexpr inline T& x() { return _coords[0]; }
  constexpr inline T& y() { return _coords[1]; }
  constexpr inline T& z() { return _coords[2]; }

  // setters
  inline void setX(const T& x) { _coords[0] = x; }
  inline void setY(const T& y) { _coords[1] = y; }
  inline void setZ(const T& z) { _coords[2] = z; }
  inline void set(const T& x, const T& y, const T& z) {
    _coords[0] = x;
    _coords[1] = y;
    _coords[2] = z;
  }

  // methods
  Vector3D<T>& normalize();

  // static methods
  static Vector3D<T> crossProduct(const Vector3D& vec1, const Vector3D& vec2);
  static T dotProduct(const Vector3D& vec1, const Vector3D& vec2);
  static T cosAngle(const Vector3D& vec1, const Vector3D& vec2);

  // friend functions
  template <typename U>
  friend std::ostream& operator<<(std::ostream& os, const Vector3D<U>& vector);
};

// constructors
template <typename T>
Vector3D<T>::Vector3D(const Vector3D<T>& other) {
  for (size_t i = 0; i < 3; i++)
    _coords[i] = other._coords[i];
}

template <typename T>
Vector3D<T>::Vector3D(Vector3D&& other) noexcept {
  for (size_t i = 0; i < 3; i++)
    _coords[i] = other._coords[i];
}

// operators
template <typename T>
Vector3D<T>& Vector3D<T>::operator=(const Vector3D<T>& other) {
  if (this != &other) {
    for (size_t i = 0; i < 3; i++)
      _coords[i] = other._coords[i];
  }

  return *this;
}

template <typename T>
Vector3D<T>& Vector3D<T>::operator=(Vector3D<T>&& other) noexcept {
  if (this != &other) {
    for (size_t i = 0; i < 3; i++)
      _coords[i] = other._coords[i];
  }

  return *this;
}

template <typename T>
inline T& Vector3D<T>::operator[](int i) {
  assert(-1 < i && i < 3);
  return _coords[i];
}

template <typename T>
Vector3D<T> Vector3D<T>::operator+(const Vector3D<T>& other) const {
  return Vector3D<T>(_coords[0] + other._coords[0],
                     _coords[1] + other._coords[1],
                     _coords[2] + other._coords[2]);
}

template <typename T>
Vector3D<T> Vector3D<T>::operator-(const Vector3D<T>& other) const {
  return Vector3D<T>(_coords[0] - other._coords[0],
                     _coords[1] - other._coords[1],
                     _coords[2] - other._coords[2]);
}

template <typename T>
Vector3D<T> Vector3D<T>::operator*(const T& value) const {
  return Vector3D<T>(_coords[0] * value, _coords[1] * value,
                     _coords[2] * value);
}

template <typename T>
Vector3D<T> Vector3D<T>::operator/(const T& value) const {
  return Vector3D<T>(_coords[0] / value, _coords[1] / value,
                     _coords[2] / value);
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator+=(const Vector3D<T>& vector) {
  _coords[0] += vector._coords[0];
  _coords[1] += vector._coords[1];
  _coords[2] += vector._coords[2];
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator+=(const T& value) {
  _coords[0] += value;
  _coords[1] += value;
  _coords[2] += value;
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator-=(const Vector3D<T>& vector) {
  _coords[0] -= vector._coords[0];
  _coords[1] -= vector._coords[1];
  _coords[2] -= vector._coords[2];
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator-=(const T& value) {
  _coords[0] -= value;
  _coords[1] -= value;
  _coords[2] -= value;
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator*=(const Vector3D<T>& vector) {
  _coords[0] *= vector._coords[0];
  _coords[1] *= vector._coords[1];
  _coords[2] *= vector._coords[2];
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator*=(const T& value) {
  _coords[0] *= value;
  _coords[1] *= value;
  _coords[2] *= value;
  return *this;
}

template <typename T>
inline Vector3D<T>& Vector3D<T>::operator/=(const Vector3D<T>& vector) {
  _coords[0] /= vector._coords[0];
  _coords[1] /= vector._coords[1];
  _coords[2] /= vector._coords[2];
  return *this;
}

template <typename T>
inline T Vector3D<T>::operator[](int i) const {
  assert(-1 < i && i < 3);
  return _coords[i];
}

// getters
template <typename T>
T Vector3D<T>::length() const {
  const T x = _coords[0];
  const T y = _coords[1];
  const T z = _coords[2];

  return sqrt(x * x + y * y + z * z);
}

// methods
template <typename T>
Vector3D<T>& Vector3D<T>::normalize() {
  const T norm = 1 / length();

  _coords[0] *= norm;
  _coords[1] *= norm;
  _coords[2] *= norm;

  return *this;
}

// static methods
template <typename T>
Vector3D<T> Vector3D<T>::crossProduct(const Vector3D<T>& a,
                                      const Vector3D<T>& b) {
  return Vector3D<T>(a.y() * b.z() - a.z() * b.y(),
                     a.z() * b.x() - a.x() * b.z(),
                     a.x() * b.y() - a.y() * b.x());
}

template <typename T>
T Vector3D<T>::dotProduct(const Vector3D& vec1, const Vector3D& vec2) {
  return vec1.x() * vec2.x() + vec1.y() * vec2.y() + vec1.z() * vec2.z();
}

template <typename T>
T Vector3D<T>::cosAngle(const Vector3D& vec1, const Vector3D& vec2) {
  return dotProduct(vec1, vec2) / (vec1.length() * vec2.length());
}

// friend functions
template <typename U>
std::ostream& operator<<(std::ostream& os, const Vector3D<U>& vector) {
  os << "[" << vector._coords[0] << "; " << vector._coords[1] << "; "
     << vector._coords[2] << "]";
  return os;
}
#endif  // VECTOR_3D_HPP
