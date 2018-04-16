#ifndef _CONTROL_SYSTEM_STATESPACE_H_
#define _CONTROL_SYSTEM_STATESPACE_H_

#include "linalg.h"

namespace control { namespace system {

using namespace linalg;

template<typename T, size_t N, size_t P=1, size_t Q=1, typename S = const T, typename M = const mat, typename V = const vec>
class Statespace {
    
    using M_A = M<S,N,N>;
    using M_B = M<S,N,P>;
    using M_C = M<S,Q,N>;
    using M_D = M<S,Q,P>;

    using V_X  = V<T, N>;
    using V_U  = V<T, P>;
    using V_Y  = V<T, Q>;

    using 

 public:
    Statespace(
               std::array<std::array<V,N>,N> A,
               std::array<std::array<V,P>,N> B,
               std::array<std::array<V,N>,Q> C,
               std::array<std::array<V,P>,Q> D
               ) : A(A), B(B), C(C), D(D) {};

    V_Y step(V_U u)
    {
      x = mul(A,x) + mul(B,u);
      return mul(C,x) + mul(D,u);
    }

 protected:
    M_A A;
    M_B B;
    M_C C;
    M_D D;

    V_X x;
}

} }

#endif _CONTROL_SYSTEM_STATESPACE_H
