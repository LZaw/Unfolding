
#pragma once

// C++ Headers
#include <utility>

// Eigen Headers
#include <Eigen/Geometry>

class DerivedQuaterniond: public Eigen::Quaterniond{
public:
  DerivedQuaterniond(): Eigen::Quaterniond(){}
  DerivedQuaterniond(Eigen::Quaterniond&& other): Eigen::Quaterniond(other){}
  DerivedQuaterniond(const Coefficients& coeffs): Eigen::Quaterniond(coeffs){}
  DerivedQuaterniond(Coefficients&& coeffs): Eigen::Quaterniond(std::move(coeffs)){}
  DerivedQuaterniond(const double w, const double x, const double y, const double z): Eigen::Quaterniond(w, x, y, z){}

  DerivedQuaterniond conjugate() const{
    return DerivedQuaterniond(Eigen::Quaterniond::conjugate());
  }
  DerivedQuaterniond operator+(double s) const{
    Coefficients coeffs(m_coeffs);
    coeffs.coeffRef(3) += s;
    return DerivedQuaterniond(coeffs);
  }
  DerivedQuaterniond operator+(const DerivedQuaterniond& other) const{
    return DerivedQuaterniond(m_coeffs + other.m_coeffs);
  }
  DerivedQuaterniond operator-(const DerivedQuaterniond& other) const{
    return DerivedQuaterniond(m_coeffs - other.m_coeffs);
  }
  DerivedQuaterniond operator*(double s) const{
    return DerivedQuaterniond(m_coeffs * s);
  }
  DerivedQuaterniond operator*(const DerivedQuaterniond& other) const{
    return DerivedQuaterniond
    (
      w() * other.w() - x() * other.x() - y() * other.y() - z() * other.z(),
      w() * other.x() + x() * other.w() + y() * other.z() - z() * other.y(),
      w() * other.y() + y() * other.w() + z() * other.x() - x() * other.z(),
      w() * other.z() + z() * other.w() + x() * other.y() - y() * other.x()
    );
  }
  DerivedQuaterniond operator/(double s) const{
    return DerivedQuaterniond(m_coeffs / s);
  }
  void operator-=(const DerivedQuaterniond& other){
    m_coeffs -= other.m_coeffs;
  }
  void operator*=(const double s){
    m_coeffs *= s;
  }
  void operator/=(const double s){
    m_coeffs /= s;
  }
};
