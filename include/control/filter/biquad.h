/*
 * biquad.h
 *
 * Bi-quadratic filters
 *
 * @author Tom Lankhorst
*/

#ifndef CONTROL_FILTER_BIQUAD_H_
#define CONTROL_FILTER_BIQUAD_H_

#include <complex>
#include <tuple>
#include <array>
#include "control/system/type.h"

namespace control { namespace filter {

  template <typename T = float>
  using TC = std::complex<T>;

  template <typename T = float>
  using TCS = std::tuple<TC<T>, TC<T>>;

  /**
   * Biquad
   *
   * Filters that - in the z domain - are the ratio of two quadratic functions.
   *
   * The general form is:
   *
   *            b0 + b1 z^-1 + b2 z^-2
   *    H(z) = ----------------------
   *            a0 + a1 z^-1 + a2 z^-2
   *
   *  Normalized by dividing all coefficients by a0.
   *
   * @tparam T arithmetic type
   * @tparam S storage type (const T)
   */
  template<typename T = float, typename S = const T>
  class Biquad : public system::SISO<T> {
   public:
    /**
     * Initialize a biquad filter with normalized (5) coefficients
     *
     * @param T b0
     * @param T b1
     * @param T b2
     * @param T a1
     * @param T a2
     */
    Biquad(T b0, T b1, T b2, T a1, T a2) : B{b0, b1, b2}, A{a1, a2} {};

    /**
     * Initialize a biquad with unnormalized (6) coefficients
     * @param b0
     * @param b1
     * @param b2
     * @param a0
     * @param a1
     * @param a2
     */
    Biquad(T b0, T b1, T b2, T a0, T a1, T a2) : B{b0/a0, b1/a0, b2/a0}, A{a1/a0, a2/a0} {};

    /**
     * Initialize a biquad with ZPK
     *
     * @param TCS<T> z zeros
     * @param TCS<T> p poles
     * @param T k gain
     */
    Biquad(TCS<T> z, TCS<T> p, T k)
    {
      static_assert(!std::is_const<S>::value, "Storage type S must be non-const to use ZPK.");
      std::tie(B[0], B[1], B[2], A[0], A[1]) = zpk2coef(z,p,k);
    };

    /**
     * Step the biquad
     *
     * @param T x input value
     * @return T output value
     */
    T step(T x)
    {
      T y;

      /* Direct form II transposed */
      y     = x * B[0]            + wz[0];
      wz[0] = x * B[1] - A[0] * y + wz[1];
      wz[1] = x * B[2] - A[1] * y;

      return y;
    }

    /**
     * Poles of the biquad
     *
     * Get a tuple of two complex values that represent the biquads poles.
     * Poles of a biquad are the solution to the denominator, a quadratic equation.
     *
     * @return TCS<T>
     */
    TCS<T> poles()
    {
      return solve((T)1, A[0], A[1]);
    }

    /**
     * Zeros of the biquad
     *
     * @see poles()
     * @return TCS<T>
     */
    TCS<T> zeros()
    {
      return solve(B[0], B[1], B[2]);
    }

    /**
     * Stability of the biquad
     *
     * Checks stability through evaluating the magnitude of the poles
     *
     * @return bool whether stable
     */
    bool stable()
    {
      auto p = poles();

      // Whether the magnitude of both poles is leq unity
      return abs(std::get<0>(p)) <= (T)1
          && abs(std::get<1>(p)) <= (T)1;
    }

   protected:

    /**
     * State variables
     * @var T[]
     */
    T wz[2] = {0,0};

    /**
     * Coefficients B
     * @var T[]
     */
    S B[3];

    /**
     * Coefficients A
     * @var T[]
     */
    S A[2];

    /**
     * Convert ZPK to coefs
     */
    std::tuple<T,T,T,T,T> zpk2coef(TCS<T> z, TCS<T> p, T k)
    {
      auto b = zero2coef(z);
      auto a = zero2coef(p);
      return std::make_tuple(
          k,
          k*std::get<0>(b),
          k*std::get<1>(b),
          std::get<0>(a),
          std::get<1>(a)
      );
    }

    /**
     * Coefficients of quadratic poly. corresponding to zeros
     *
     * Gets the coefficients of the quadratic polynomial s.t. its solutions are p
     *
     * (x-z1)(x-z2) -> x^2-(z1+z2)+z1*z2
     *
     * @param TCS<T> z tuple of two complex Ts
     * @return std::tuple<T,T> tuple of two Ts
     */
    std::tuple<T,T> zero2coef(TCS<T> z)
    {
      auto z1 = std::get<0>(z);
      auto z2 = std::get<1>(z);

      auto z1r = z1.real();
      auto z1i = z1.imag();
      auto z2r = z2.real();

      // Assume this is a complex conjugate pair if p1 has imag part
      return z1i
             ? std::make_tuple(-2*z1r, z1r*z1r+z1i*z1i)
             : std::make_tuple(-z1r-z2r, z1r*z2r);
    };

    /**
     * Solve a quadratic polynomial
     *
     * ax^2+bx+c -> k(x-z1)(x-z2)
     *
     * @param a T
     * @param b T
     * @param c T
     * @return TCS<T> zeros
     */
    TCS<T> solve(T a, T b, T c)
    {
      // Normalize
      b /= a;
      c /= a;

      // sqrt(b^2 - 4*a*c)
      TC<T> ds = std::sqrt( TC<T>(b*b,0)-4*c );

      // (-b±ds)/2a
      return std::make_tuple<TC<T>, TC<T>>((-b+ds)/(T)2, (-b-ds)/(T)2);
     }

  };


  /**
   * Type traits for deducing the typename of T and S of a biquad
   * @tparam T
   */
  template<typename T>
  struct inspect_types
  {
    typedef T arithmetic_type;
    typedef T storage_type;
  };

  template<template<typename, typename> class B, typename T, typename S>
  struct inspect_types<B<T,S>>
  {
    typedef T arithmetic_type;
    typedef S storage_type;
  };

  /**
   * Cascade of biquads
   *
   * Helps in realizing higher-order filters
   *
   * @tparam N number of sections
   * @tparam B Biquad class
   */
  template<typename B=Biquad<>, size_t N = 1>
  class BiquadCascade : public system::SISO<typename inspect_types<B>::arithmetic_type> {
    using T = typename inspect_types<B>::arithmetic_type;
    using BS = std::array<B,N>;
   public:

    /**
     * Initialize chain with biquads
     *
     * @param bs biquads
     */
    template<typename... T>
    BiquadCascade(T... bs) : bs{bs...} {}

    /**
     * Step the biquad chain one time
     *
     * @param u T input
     * @return T output
     */
    T step(T u)
    {
      for( auto &b : bs )
        u = b.step(u);

      return u;
    }

   protected:

    /**
     * Container of biquads
     */
    BS bs;
  };

} }

#endif /* CONTROL_FILTER_BIQUAD_H_ */

