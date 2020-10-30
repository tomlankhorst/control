/**
 * System Identification Signals
 */
#pragma once

#include <random>

namespace control::ident {

/**
 * PRBS
 *
 * Pseudo-Random-Binary-Signal
 *
 * Generates a sequence of -1 and 1
 *
 * @tparam T
 */
template<typename T>
class PRBS {
 public:

  /**
   * Initialize the PRBS
   */
  PRBS() : e(), d(0, 1) {};

  /**
   * Get an integer, either -1 or 1
   *
   * @return T number in {-1,1}
   */
  T get() {
    return d(e) ? 1 : -1;
  }
 protected:
  std::default_random_engine e;
  // The distribution still uses ints
  std::uniform_int_distribution<int> d;
};

}
