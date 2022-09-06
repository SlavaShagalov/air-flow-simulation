#ifndef MATRIX_4X4_HPP
#define MATRIX_4X4_HPP

#define DEPTH 255

#include <cassert>
#include <cmath>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <Domains/Applied/Primitives/Vector4D/vector_4d.hpp>

template <typename T>
class Matrix4x4;

using Mtr4x4i = Matrix4x4<int>;
using Mtr4x4f = Matrix4x4<float>;
using Mtr4x4d = Matrix4x4<double>;

// row-major
template <typename T>
class Matrix4x4 {
  static_assert(std::is_arithmetic<T>::value, "Type must be arithmetic.");

private:
  T _mtr[4][4];

public:
  // constructors
  inline Matrix4x4() { setToIdentity(); }
  Matrix4x4(const Matrix4x4& other);
  Matrix4x4(Matrix4x4&& other) noexcept;

  constexpr Matrix4x4(std::initializer_list<T> initList) : _mtr(initList) {}
  inline Matrix4x4(T m11, T m12, T m13, T m14, T m21, T m22, T m23, T m24,
                   T m31, T m32, T m33, T m34, T m41, T m42, T m43, T m44);

  ~Matrix4x4() = default;

  // operators
  Matrix4x4& operator=(const Matrix4x4<T>& other);
  Matrix4x4& operator=(Matrix4x4<T>&& other) noexcept;

  Matrix4x4<T> operator*(const Matrix4x4& other) const;
  Vector4D<T> operator*(const Vector4D<T>& vec) const;

  //  T& operator[](int i);
  //  T operator[](int i) const;

  // getters

  // setters

  // methods
  inline bool isIdentity() const;
  inline void setToIdentity();
  inline void fill(const T& value);

  // static methods
  static Matrix4x4<T> lookAt(const Vector3D<T>& eye, const Vector3D<T>& center,
                             const Vector3D<T>& up);
  static Matrix4x4<T> viewport(const T x, const T y, const T width,
                               const T height);

  // friend method
  template <class U>
  friend Vector4D<U> operator*(const Vector4D<U>& vec, const Matrix4x4<U>& mtr);

  template <typename U>
  friend std::ostream& operator<<(std::ostream& os, const Matrix4x4<U>& mtr);
};

// constructors
template <typename T>
Matrix4x4<T>::Matrix4x4(const Matrix4x4<T>& other) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      _mtr[i][j] = other._mtr[i][j];
}

template <typename T>
Matrix4x4<T>::Matrix4x4(Matrix4x4<T>&& other) noexcept {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      _mtr[i][j] = other._mtr[i][j];
}

template <typename T>
inline Matrix4x4<T>::Matrix4x4(T m11, T m12, T m13, T m14, T m21, T m22, T m23,
                               T m24, T m31, T m32, T m33, T m34, T m41, T m42,
                               T m43, T m44) {
  _mtr[0][0] = m11;
  _mtr[0][1] = m12;
  _mtr[0][2] = m13;
  _mtr[0][3] = m14;
  _mtr[1][0] = m21;
  _mtr[1][1] = m22;
  _mtr[1][2] = m23;
  _mtr[1][3] = m24;
  _mtr[2][0] = m31;
  _mtr[2][1] = m32;
  _mtr[2][2] = m33;
  _mtr[2][3] = m34;
  _mtr[3][0] = m41;
  _mtr[3][1] = m42;
  _mtr[3][2] = m43;
  _mtr[3][3] = m44;
}

// operators
template <typename T>
Matrix4x4<T>& Matrix4x4<T>::operator=(const Matrix4x4<T>& other) {
  if (this != &other) {
    _mtr = other._mtr;
  }

  return *this;
}

template <typename T>
Vector4D<T> Matrix4x4<T>::operator*(const Vector4D<T>& vec) const {
  Vector4D<T> res(0, 0, 0, 0);

  for (int i = 0; i < 4; ++i) {
    res[i] = _mtr[i][0] * vec[0] + _mtr[i][1] * vec[1] + _mtr[i][2] * vec[2] +
             _mtr[i][3] * vec[3];
  }

  return res;
}

template <typename T>
Matrix4x4<T>& Matrix4x4<T>::operator=(Matrix4x4<T>&& other) noexcept {
  if (this != &other) {
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        _mtr[i][j] = other[i][j];
  }

  return *this;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::operator*(const Matrix4x4<T>& other) const {
  Matrix4x4<T> res;

  for (int i = 0; i < 4; ++i) {
    res._mtr[i][0] =
    _mtr[i][0] * other._mtr[0][0] + _mtr[i][1] * other._mtr[1][0] +
    _mtr[i][2] * other._mtr[2][0] + _mtr[i][3] * other._mtr[3][0];

    res._mtr[i][1] =
    _mtr[i][0] * other._mtr[0][1] + _mtr[i][1] * other._mtr[1][1] +
    _mtr[i][2] * other._mtr[2][1] + _mtr[i][3] * other._mtr[3][1];

    res._mtr[i][2] =
    _mtr[i][0] * other._mtr[0][2] + _mtr[i][1] * other._mtr[1][2] +
    _mtr[i][2] * other._mtr[2][2] + _mtr[i][3] * other._mtr[3][2];

    res._mtr[i][3] =
    _mtr[i][0] * other._mtr[0][3] + _mtr[i][1] * other._mtr[1][3] +
    _mtr[i][2] * other._mtr[2][3] + _mtr[i][3] * other._mtr[3][3];
  }

  //  res.m[0][0] = _mtr[0][0] * other._mtr[0][0] + _mtr[1][0] * other.m[0][1] +
  //                _mtr[2][0] * other._mtr[0][2] + _mtr[3][0] * other.m[0][3];
  //  res.m[0][1] = _mtr[0][1] * other._mtr[0][0] + _mtr[1][1] * other.m[0][1] +
  //                _mtr[2][1] * other._mtr[0][2] + _mtr[3][1] * other.m[0][3];
  //  res.m[0][2] = _mtr[0][2] * other._mtr[0][0] + _mtr[1][2] * other.m[0][1] +
  //                _mtr[2][2] * other._mtr[0][2] + _mtr[3][2] * other.m[0][3];
  //  res.m[0][3] = _mtr[0][3] * other._mtr[0][0] + _mtr[1][3] * other.m[0][1] +
  //                _mtr[2][3] * other._mtr[0][2] + _mtr[3][3] * other.m[0][3];

  //  res.m[1][0] = _mtr[0][0] * other._mtr[1][0] + _mtr[1][0] *
  //  other._mtr[1][1] +
  //                _mtr[2][0] * other._mtr[1][2] + _mtr[3][0] *
  //                other._mtr[1][3];
  //  res.m[1][1] = _mtr[0][1] * other._mtr[1][0] + _mtr[1][1] *
  //  other._mtr[1][1] +
  //                _mtr[2][1] * other._mtr[1][2] + _mtr[3][1] *
  //                other._mtr[1][3];
  //  res.m[1][2] = _mtr[0][2] * other._mtr[1][0] + _mtr[1][2] *
  //  other._mtr[1][1] +
  //                _mtr[2][2] * other._mtr[1][2] + _mtr[3][2] *
  //                other._mtr[1][3];
  //  res.m[1][3] = _mtr[0][3] * other._mtr[1][0] + _mtr[1][3] *
  //  other._mtr[1][1] +
  //                _mtr[2][3] * other._mtr[1][2] + _mtr[3][3] *
  //                other._mtr[1][3];

  //  res.m[2][0] = _mtr[0][0] * other._mtr[2][0] + _mtr[1][0] *
  //  other._mtr[2][1] +
  //                _mtr[2][0] * other._mtr[2][2] + _mtr[3][0] *
  //                other._mtr[2][3];
  //  res.m[2][1] = _mtr[0][1] * other._mtr[2][0] + _mtr[1][1] *
  //  other._mtr[2][1] +
  //                _mtr[2][1] * other._mtr[2][2] + _mtr[3][1] *
  //                other._mtr[2][3];
  //  res.m[2][2] = _mtr[0][2] * other._mtr[2][0] + _mtr[1][2] *
  //  other._mtr[2][1] +
  //                _mtr[2][2] * other._mtr[2][2] + _mtr[3][2] *
  //                other._mtr[2][3];
  //  res.m[2][3] = _mtr[0][3] * other._mtr[2][0] + _mtr[1][3] *
  //  other._mtr[2][1] +
  //                _mtr[2][3] * other._mtr[2][2] + _mtr[3][3] *
  //                other._mtr[2][3];

  //  res.m[3][0] = _mtr[0][0] * other._mtr[3][0] + _mtr[1][0] *
  //  other._mtr[3][1] +
  //                _mtr[2][0] * other._mtr[3][2] + _mtr[3][0] *
  //                other._mtr[3][3];
  //  res.m[3][1] = _mtr[0][1] * other._mtr[3][0] + _mtr[1][1] *
  //  other._mtr[3][1] +
  //                _mtr[2][1] * other._mtr[3][2] + _mtr[3][1] *
  //                other._mtr[3][3];
  //  res.m[3][2] = _mtr[0][2] * other._mtr[3][0] + _mtr[1][2] *
  //  other._mtr[3][1] +
  //                _mtr[2][2] * other._mtr[3][2] + _mtr[3][2] *
  //                other._mtr[3][3];
  //  res.m[3][3] = _mtr[0][3] * other._mtr[3][0] + _mtr[1][3] *
  //  other._mtr[3][1] +
  //                _mtr[2][3] * other._mtr[3][2] + _mtr[3][3] *
  //                other._mtr[3][3];

  return res;
}

// getters

// methods
template <typename T>
inline bool Matrix4x4<T>::isIdentity() const {
  if (_mtr[0][0] != 1 || _mtr[0][1] != 0 || _mtr[0][2] != 0 ||
      _mtr[0][3] != 0 || _mtr[1][0] != 0 || _mtr[1][1] != 1 ||
      _mtr[1][2] != 0 || _mtr[1][3] != 0 || _mtr[2][0] != 0 ||
      _mtr[2][1] != 0 || _mtr[2][2] != 1 || _mtr[2][3] != 0 ||
      _mtr[3][0] != 0 || _mtr[3][1] != 0 || _mtr[3][2] != 0 || _mtr[3][3] == 1)
    return false;
  return true;
}

template <typename T>
inline void Matrix4x4<T>::setToIdentity() {
  _mtr[0][0] = 1;
  _mtr[0][1] = 0;
  _mtr[0][2] = 0;
  _mtr[0][3] = 0;
  _mtr[1][0] = 0;
  _mtr[1][1] = 1;
  _mtr[1][2] = 0;
  _mtr[1][3] = 0;
  _mtr[2][0] = 0;
  _mtr[2][1] = 0;
  _mtr[2][2] = 1;
  _mtr[2][3] = 0;
  _mtr[3][0] = 0;
  _mtr[3][1] = 0;
  _mtr[3][2] = 0;
  _mtr[3][3] = 1;
}

template <typename T>
inline void Matrix4x4<T>::fill(const T& value) {
  _mtr[0][0] = value;
  _mtr[0][1] = value;
  _mtr[0][2] = value;
  _mtr[0][3] = value;
  _mtr[1][0] = value;
  _mtr[1][1] = value;
  _mtr[1][2] = value;
  _mtr[1][3] = value;
  _mtr[2][0] = value;
  _mtr[2][1] = value;
  _mtr[2][2] = value;
  _mtr[2][3] = value;
  _mtr[3][0] = value;
  _mtr[3][1] = value;
  _mtr[3][2] = value;
  _mtr[3][3] = value;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::lookAt(const Vector3D<T>& eye,
                                  const Vector3D<T>& center,
                                  const Vector3D<T>& up) {
  Vector3D<T> zVec = (eye - center).normalize();
  Vector3D<T> xVec = Vector3D<T>::crossProduct(up, zVec).normalize();
  Vector3D<T> yVec = Vector3D<T>::crossProduct(zVec, xVec).normalize();

  Matrix4x4<T> translate = Matrix4x4<T>(1, 0, 0, -center[0],  //
                                        0, 1, 0, -center[1],  //
                                        0, 0, 1, -center[2],  //
                                        0, 0, 0, 1);

  Matrix4x4<T> Minv = Matrix4x4<T>(xVec.x(), xVec.y(), xVec.z(), 0,  //
                                   yVec.x(), yVec.y(), yVec.z(), 0,  //
                                   zVec.x(), zVec.y(), zVec.z(), 0,  //
                                   0, 0, 0, 1);
  return Minv * translate;
}

template <typename T>
Matrix4x4<T> Matrix4x4<T>::viewport(const T x, const T y, const T width,
                                    const T height) {
  return Matrix4x4<T>((float)width / 2.0, 0, 0, x + (float)width / 2.0,    //
                      0, (float)height / 2.0, 0, y + (float)height / 2.0,  //
                      0, 0, (float)DEPTH / 2.0, (float)DEPTH / 2.0,        //
                      0, 0, 0, 1);

  //  return Matrix4x4<T>((float)width / 2.0, 0, 0, x + (float)width / 2.0, //
  //                      0, (float)height / 2.0, 0, y + (float)height / 2.0, //
  //                      0, 0, 1, 0, // 0, 0, 0, 1);
}

// static methods

// friend functions
template <class U>
Vector4D<U> operator*(const Vector4D<U>& vec, const Matrix4x4<U>& mtr) {
  Vector4D<U> res(0, 0, 0, 0);

  for (int i = 0; i < 4; ++i) {
    res[i] = vec[0] * mtr._mtr[0][i] + vec[1] * mtr._mtr[1][i] +
             vec[2] * mtr._mtr[2][i] + vec[3] * mtr._mtr[3][i];
  }

  return res;
}

template <typename U>
std::ostream& operator<<(std::ostream& os, const Matrix4x4<U>& mtr) {
  for (int i = 0; i < 4; ++i) {
    os << mtr._mtr[i][0] << "; " << mtr._mtr[i][1] << "; " << mtr._mtr[i][2]
       << "; " << mtr._mtr[i][3] << std::endl;
  }

  return os;
}
#endif  // MATRIX_4X4_HPP
