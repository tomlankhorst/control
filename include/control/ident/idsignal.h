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
  PRBS() : e(), d(0,1) {};

  /**
   * Get a number
   *
   * @return T number
   */
  T get() {
    return d(e)*2-1;
  }
 protected:
  std::default_random_engine e;
  std::uniform_int_distribution<T> d;
};

} }

#endif //CONTROL_IDENT_IDSIGNAL_H
