C++ Control
===========

[![Build Status](https://travis-ci.org/tomlankhorst/control.svg?branch=master)](https://travis-ci.org/tomlankhorst/control)

Control functionality in C++

Classic
-------

P, PI and PID control

```cpp
#include <control/classic.h>

typedef control::classic::PI<double> PI;

// ...

// Ts = 1.0s, K = 1.0, Ti = 2.0
PI controller(1.0, 1.0, 2.0);

for( int i = 0; i < 10; i++ )
  std::cout << controller.step(1.0) << std::endl;

// 1.5 2.0 2.5 ...

```

Tests
-----

```bash
mkdir build && cd build
cmake .. -DTESTS=ON
make
./controltests
```
