#ifndef CONTROL_SYSTEM_TYPE_H
#define CONTROL_SYSTEM_TYPE_H

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

#endif //CONTROL_SYSTEM_TYPE_H
