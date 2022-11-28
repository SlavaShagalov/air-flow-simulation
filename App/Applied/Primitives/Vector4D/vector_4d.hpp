#ifndef VECTOR_4D_HPP
#define VECTOR_4D_HPP

#include <cassert>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>

template <typename T>
class Vector4D;

using Vec4i = Vector4D<int>;
using Vec4f = Vector4D<float>;
using Vec4d = Vector4D<double>;

template <typename T>
class Vector4D {
  static_assert(std::is_arithmetic<T>::value, "Type must be arithmetic.");

private:
  T _coords[4];

public:
  // constructors
  constexpr inline Vector4D() : _coords{ 0, 0, 0, 0 } {}
  Vector4D(const Vector4D& other);
  Vector4D(Vector4D&& other) noexcept;

  constexpr Vector4D(const T& x, const T& y, const T& z, const T& w)
      : _coords{ x, y, z, w } {}
  constexpr Vector4D(std::initializer_list<T> initList)
      : _coords{
        *initList.begin(),
        *(initList.begin() + 1),
        *(initList.begin() + 2),
        *(initList.begin() + 3),
      } {}

  ~Vector4D() = default;

  // operators
  Vector4D& operator=(const Vector4D<T>& other);
  Vector4D& operator=(Vector4D<T>&& other) noexcept;

  T& operator[](int i);
  T operator[](int i) const;

  // getters
  T length() const;
  constexpr inline T x() const { return _coords[0]; }
  constexpr inline T y() const { return _coords[1]; }
  constexpr inline T z() const { return _coords[2]; }
  constexpr inline T w() const { return _coords[3]; }

  // setters
  inline void setX(const T& x) { _coords[0] = x; }
  inline void setY(const T& y) { _coords[1] = y; }
  inline void setZ(const T& z) { _coords[2] = z; }
  inline void setW(const T& w) { _coords[3] = w; }

  // methods
  void normalize();

  // static methods
  static T dotProduct(const Vector4D& vec1, const Vector4D& vec2);
  static T cosAngle(const Vector4D& vec1, const Vector4D& vec2);

  // friend functions
  template <typename U>
  friend std::ostream& operator<<(std::ostream& os, const Vector4D<U>& vector);
};

// constructors
template <typename T>
Vector4D<T>::Vector4D(const Vector4D<T>& other) {
  _coords = other._coords;
}

template <typename T>
Vector4D<T>::Vector4D(Vector4D&& other) noexcept {
  for (int i = 0; i < 4; i++)
    _coords[i] = other._coords[i];
}

// operators
template <typename T>
Vector4D<T>& Vector4D<T>::operator=(const Vector4D<T>& other) {
  if (this != &other) {
    _coords = other._coords;
  }

  return *this;
}

template <typename T>
Vector4D<T>& Vector4D<T>::operator=(Vector4D<T>&& other) noexcept {
  if (this != &other) {
    for (int i = 0; i < 4; i++)
      _coords[i] = other._coords[i];
  }

  return *this;
}

template <typename T>
inline T& Vector4D<T>::operator[](int i) {
  assert(-1 < i && i < 4);
  return _coords[i];
}

template <typename T>
inline T Vector4D<T>::operator[](int i) const {
  assert(-1 < i && i < 4);
  return _coords[i];
}

// getters
template <typename T>
T Vector4D<T>::length() const {
  const T x = _coords[0];
  const T y = _coords[1];
  const T z = _coords[2];
  const T w = _coords[3];

  return sqrt(x * x + y * y + z * z + w * w);
}

// methods
template <typename T>
void Vector4D<T>::normalize() {
  const T norm = 1 / length();

  _coords[0] *= norm;
  _coords[1] *= norm;
  _coords[2] *= norm;
  _coords[3] *= norm;
}

// static methods
template <typename T>
T Vector4D<T>::dotProduct(const Vector4D& vec1, const Vector4D& vec2) {
  return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2] +
         vec1[3] * vec2[3];
}

template <typename T>
T Vector4D<T>::cosAngle(const Vector4D& vec1, const Vector4D& vec2) {
  return dotProduct(vec1, vec2) / (vec1.length() * vec2.length());
}

// friend functions
template <typename U>
std::ostream& operator<<(std::ostream& os, const Vector4D<U>& vector) {
  os << "[" << vector._coords[0] << "; " << vector._coords[1] << "; "
     << vector._coords[2] << "; " << vector._coords[3] << "]";
  return os;
}
#endif  // VECTOR_4D_HPP
