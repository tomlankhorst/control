#ifndef CONTROL_IDENT_IDSIGNAL_H
#define CONTROL_IDENT_IDSIGNAL_H

namespace control { namespace system {

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

} }

#endif //CONTROL_IDENT_IDSIGNAL_H
