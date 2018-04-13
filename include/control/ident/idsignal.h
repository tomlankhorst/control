#ifndef CONTROL_IDENT_IDSIGNAL_H
#define CONTROL_IDENT_IDSIGNAL_H

#include <random>

namespace control { namespace ident {

template <typename T>
class prbs {
 public:
  prbs() :
      e(),
      d(0,1)
      {};
  int get() {
    return d(e)*2-1;
  }
 protected:
  std::default_random_engine e;
  std::uniform_int_distribution<T> d;
};

} }

#endif //CONTROL_IDENT_IDSIGNAL_H
