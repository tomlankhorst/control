C++ Control
===========

[![Build Status](https://travis-ci.org/tomlankhorst/control.svg?branch=master)](https://travis-ci.org/tomlankhorst/control)

Control functionality in C++

Classic PIDF control
-------

P, PI, PD and PID control:

```cpp
#include <control/classic/pid.h>

using P   = control::classic::P<int>;
using PI  = control::classic::PI<float>;
using PD  = control::classic::PI<double>;
using PID = control::classic::PID<double>;

// ...

// Ts = 1.0s, K = 1.0, Ti = 2.0
PI controller(1, 1, 2);

for( int i = 0; i < 10; i++ )
  std::cout << controller.step(1.0) << std::endl;

// 1.5 2.0 2.5 ...

```

Based on trapezoidal (Tustin) discretizations of the standard (serial) PID controller.

PI, PD and PID are use Biquads and expose functionality like `.poles()`. 

Biquad Digital Filters
-----

Biquads, or Second Order Sections (SOS), are transfer functions consisting of a ratio of two quadratic polynomials.
With _z_ operator:

```
       b0 + b1 z^-1 + b2 z^-2
H(z) = ----------------------
        1 + a1 z^-1 + a2 z^-2
``` 
(normalized by a0)

Biquads can be chained to obtain higher-order transfer-functions. 

The implementation of Biquads in this library is based on the _Direct-form II transposed_ implementation. 

```cpp
// 2-nd order Butterworth LP filter with Wc =~ 0.1 (* half sample-rate)
float b0 = 0.02, b1 = 0.04, b2 = 0.02;
float a1 = -1.56, a2 = 0.64;
Biquad<float> b(b0, b1, b2, a1, a2);

for(int i=0; i<5; i++)
    std::cout << b.step(1) << std::endl;
// 0.02.., 0.09.., 0.22.., ...
```

Retrieving the poles of a Biquad:

```cpp
// Tuple of two std::complex<T>
auto ps = b.poles();
// std::complex<T>
auto p1 = std::get<0>(ps);
auto p2 = std::get<1>(ps);
```

System Identification
-----

### Pseudo-Random Binary Signal

Expose a system to a PRBS to identify its transfer characteristics.

```cpp
control::ident::PRBS<int> P;
auto i = P.get(); // 1 of -1
```

State-Space Systems
-----

State-spaces allow representation of LTI discretized systems with `Nx` states, `Nu` inputs and `Ny` outputs. 

```cpp
// 2nd order SISO state-space
using ss2 = control::system::ss<float,2>;

ss2::TA A << 1, 1, 0, 1;
ss2::TA B << 0, 1;
ss2::TC C << 1, 0;
ss2::TD D << 0;
ss2 P(A,B,C,D);

// 3th order MIMO state-space
using ss3 = control::system::ss<float,3,2,2>;

ss3::TA A << /* ... */ ;
// ...

ss3 P3(A,B,C,D);

ss3::Tu u << 1, 2;
auto y = P3.step(u);
// y(0), y(1)
```

This functionality is based upon the Eigen3 Matrix math library. 
Eigen takes care of target-specific vectorization!

Tests
-----

```bash
mkdir build && cd build
cmake .. -DTESTS=ON
make
./controltests
```

There's a short [article about this library](https://tomlankhorst.nl/filtering-and-control-library/). 
