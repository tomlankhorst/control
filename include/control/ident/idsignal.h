/**
 * idsignal.h
 *
 * System Identification Signals
 *
 * @author Tom Lankhorst
 */
#ifndef CONTROL_IDENT_IDSIGNAL_H
#define CONTROL_IDENT_IDSIGNAL_H

#include <random>

namespace control { namespace ident {

/**
 * PRBS
 *
 * Pseudo-Random-Binary-Signal
 *
 * Generates a sequence of -1 and 1
 *
 * @tparam T
 */
template <typename T>
class PRBS {
 public:

  /**
   * Initialize the PRBS
   */
  PRBS() : e(s), d(0,1) {};

  /**
   * Get a number
   *
   * @return T number
   */
  T get() {
    return d(e)*2-1;
  }
 protected:
  // A Mersenne-Twister generator
  std::mt19937 e;
  // The distribution still uses ints
  std::uniform_int_distribution<int> d;
  // The seed
  std::seed_seq s{1,2,3,4,5,6,7,8};
};

} }

#endif //CONTROL_IDENT_IDSIGNAL_H
