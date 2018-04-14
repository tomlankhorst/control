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

namespace control { namespace filter {

  template <typename T>
  using TC = std::complex<T>;

  template <typename T>
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
   * @tparam T
   */
  template<typename T>
  class Biquad {
   public:
    /**
     * Initialize a biquad filter
     *
     * @param T b0
     * @param T b1
     * @param T b2
     * @param T a1
     * @param T a2
     */
    Biquad(T b0, T b1, T b2, T a1, T a2) : B{b0, b1, b2}, A{a1, a2} {};
    ~Biquad() {};

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
      // sqrt(b^2 - 4*a*c)
      TC<T> ds = std::sqrt( TC<T>(A[0]*A[0],0)-4*A[1] );

      // (-b±ds)/2a
      return std::make_tuple((-A[0]+ds)/(T)2, (-A[0]-ds)/(T)2);
    }

    /**
     * Stability of the biquad
     *
     * Checks stability through evaluating the magnitude of the poles
     *
     * @return true of stable
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
    const T B[3];

    /**
     * Coefficients A
     * @var T[]
     */
    const T A[2];
  };

} }

#endif /* CONTROL_FILTER_BIQUAD_H_ */

