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

  template<typename T>
  class Biquad {
   public:
    Biquad(T b0, T b1, T b2, T a1, T a2) : B{b0, b1, b2}, A{a1, a2} {};
    ~Biquad() {};
    T step(T x)
    {
      T y;

      /* Direct form II transposed */
      y = B[0] * x + wz[0];
      wz[0] = B[1] * x - A[0] * y + wz[1];
      wz[1] = B[2] * x - A[1] * y;

      return y;
    }
   protected:
    T wz[2] = {0,0};
    const T B[3];
    const T A[2];
  };

} }

#endif /* CONTROL_FILTER_BIQUAD_H_ */

