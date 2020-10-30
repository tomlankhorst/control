/**
 * ghk-filter, or alpha beta gamma filter.
 */

#pragma once

#include <cmath>

namespace control::ghk {

// g h k = alpha beta gamma/2
template<typename ValueType>
struct coeff {
  ValueType g, h, k;
};

// x dx ddx
template<typename ValueType>
struct state {
  ValueType x, dx, ddx;
};

namespace parameterize {

// Get g h k parameters from alpha beta gamma
template<typename ValueType>
constexpr coeff<ValueType> abc(ValueType a, ValueType b, ValueType c) {
  return { a, b, c/2 };
}

// Get g h k parameters from critical dampened theta
// Eli Brookner DSc, Tracking and Kalman Filtering Made Easy - g–h and g–h–k Filters
template<typename ValueType>
constexpr coeff<ValueType> critical_dampened(ValueType th) {
  return {
      1-std::pow(th,3),
      3*(1-th*th)*(1-th)/2,
      pow(1-th,3)/2,
  };
}

// Get g h k parameters from optimal guassian (Kalman) lambda
// J. E. Gray and W. Murray, "A derivation of an analytic expression for the tracking index for the alpha-beta-gamma filter," in IEEE Transactions on Aerospace and Electronic Systems, vol. 29, no. 3, pp. 1064-1065, July 1993, doi: 10.1109/7.220956.
template<typename ValueType>
constexpr coeff<ValueType> optimal_gaussian(ValueType l) {
  auto b = l/2-3;
  auto c = l/2+3;
  auto d = -1;
  auto p = c-b*b/3;
  auto q = 2*b*b*b/27-b*c/3+d;
  auto v = sqrt(q*q+4*p*p*p/27);
  auto z = -std::pow(q+v/2,1./3);
  auto s = z-p/(3*z)-b/3;
  auto g = 1-s*s;
  auto h = 2*s*s-4*s+2;
  auto k = h*h/(2*g)/2;
  return { g, h, k };
}

/**
 * Get g h k parameters from process and measurement noise covariance and timestep T
 * @param s_w sigma w, for process variance sigma_w^2
 * @param s_v sigma v, for measurement variance sigma_v^2
 * @param T sample-time or time-step
 */
template<typename ValueType>
constexpr coeff<ValueType> optimal_gaussian(ValueType s_w, ValueType s_v, ValueType T) {
  auto l = s_w*T*T/s_v;
  return optimal_gaussian<ValueType>(l);
}

}

template<typename ValueType>
struct result {
  state<ValueType> correction, prediction;
};

template<typename ValueType>
result<ValueType> correct_predict(const coeff<ValueType>& coeff, state<ValueType> current, ValueType z, ValueType T) {
  auto& [g,h,k] = coeff;

  // update with residual
  auto r = z - current.x;

  current.x += g*r;
  current.dx += h/T*r;
  current.ddx += 2*k/(T*T)*r;

  auto correct = current;

  // predict with current value
  current.x += current.dx*T+current.ddx*T*T/2;
  current.dx += current.ddx*T;

  return {correct, current };
}

}
