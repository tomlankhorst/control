/*
 * biquad.h
 *
 * Bi-quadratic filters
 *
 * @author Tom Lankhorst
*/

#ifndef CONTROL_FILTER_BIQUAD_H_
#define CONTROL_FILTER_BIQUAD_H_

namespace control { namespace filter {

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

