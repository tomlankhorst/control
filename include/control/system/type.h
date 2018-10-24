#ifndef CONTROL_IDENT_IDSIGNAL_H
#define CONTROL_IDENT_IDSIGNAL_H

namespace control {
namespace system {

/**
 * Single-input-single-output
 * A state-space that takes a single input and produces a single output.
 * A SISO system has mathematical equivalents like PZKs, TFs and BiQuads (SOSs).
 * @tparam T
 */
template<typename T>
class SISO {
 public:
  /**
   * Step the SISO filter one step
   *
   * @param T input
   * @return T output
   */
  virtual T step(T) = 0;
};

}
}

#endif //CONTROL_IDENT_IDSIGNAL_H
