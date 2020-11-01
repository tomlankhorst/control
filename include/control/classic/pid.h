/*
 * Classic P/PI/PID control
 */

#pragma once

#include <limits>
#include <algorithm>

#include "control/filter/biquad.h"
#include "control/system/type.h"

namespace control::classic {

/**
 * Helper that returns the maximum value of the type or infinity when present
 * @tparam T
 * @return
 */
template<typename T>
constexpr T max() {
  return std::numeric_limits<T>::has_infinity
         ? std::numeric_limits<T>::infinity() : std::numeric_limits<T>::max();
}

/**
 * Base controller that implements limiting, resetting and stepping
 * @tparam T
 */
template<typename T>
class AbstractController : public system::SISO<T> {
  static_assert(std::numeric_limits<T>::is_signed, "Signed type required");
 public:
  /**
   * Constructor
   * @param Limit_ maximum value of the output signal
   */
  explicit AbstractController(T Limit_ = max<T>()) : Limit(Limit_) {};

  virtual ~AbstractController() = default;

  // Limit
  T Limit;

  // Clipping status
  bool clipping = false;

  /**
   * Steps the controller one time-step
   *
   * @param T e the error value
   * @return T the controller output
   */
  T step(T e) final {
    T u;

    // Get the control effort
    u = control(e);

    // Clip the output
    u = clip(u);

    // Return the output
    return u;
  };

  /**
   * Update the output limit
   *
   * @param T limit
   */
  void setLimit(T limit) {
    Limit = limit;
  }

  /**
   * Reset the state of the controller
   */
  virtual void reset() {};

 protected:

  /**
   * Limit the output
   *
   * @param T u
   * @return T
   */
  T clip(T u) {
    if (Limit == std::numeric_limits<T>::infinity()) {
      return u;
    }

    // Determine if the signal is in range
    if (!clipping && (u > Limit || u < -Limit)) {
      clipping = true;
    } else if (clipping && (u >= -Limit && u <= Limit)) {
      clipping = false;
    }

    // If limiting, then clip to min / max
    if (clipping) {
      u = std::max(-Limit, std::min(Limit, u));
    }

    return u;
  }

  /**
   * Retrieves the control output, unclipped
   *
   * @param T e the error
   * @return T the raw output
   */
  virtual T control(T e) = 0;

};

/**
 * Proportional controller
 */
template<typename T>
class P : public AbstractController<T> {
 public:

  /**
   * Constructor of proportional controller
   *
   * @param Kp_ Proportional gain
   * @param Limit_ Maximum output value
   */
  explicit P(T Kp_ = 1.0, T Limit_ = max<T>()) : AbstractController<T>(Limit_), Kp(Kp_) {};

  ~P() = default;;

  // Proportional gain
  const T Kp;

 protected:
  /**
   * @inheritdoc
   */
  T control(T e) {
    // The output of the controller
    T u;

    // Proportional gain
    u = Kp * e;

    return u;
  };
};

/**
 * Proportional integral derivative controller
 */
template<typename T>
class PID : public AbstractController<T> {
 public:

  /**
   * Constructor of PID controller
   * @param Ts Timestep (s)
   * @param Kp Proportional gain
   * @param Ti Integrator time-constant (s)
   * @param Td Differentiator time-constant (s)
   * @param N Filter coefficicent
   * @param Limit Maximum output
   */
  explicit PID(T Ts = 1.0, T Kp = 1.0, T Ti = max<T>(), T Td = 0.0, T N = max<T>(), T Limit = max<T>())
      : AbstractController<T>(Limit), B(
      (Kp * (4 * Td / N + 2 * Td * Ts / Ti / N + Ts * Ts / Ti + 4 * Td + 2 * Ts)) / (4 * Td / N + 2 * Ts),
      -(Kp * (-Ts * Ts / Ti + 4 * Td / N + 4 * Td)) / (2 * Td / N + Ts),
      (Kp * (4 * Td / N - 2 * Td * Ts / Ti / N + Ts * Ts / Ti + 4 * Td - 2 * Ts)) / (4 * Td / N + 2 * Ts),
      -(4 * Td / N) / (2 * Td / N + Ts),
      (2 * Td / N - Ts) / (2 * Td / N + Ts)
  ) {};

  /**
   * Poles of the PID controller
   *
   * @return control::filter::TCS<T> pair of complex T in tuple
   */
  filter::TCS<T> poles() {
    return B.poles();
  }

  /**
   * Reset the state of the controller
   */
  void reset() {
    B.reset();
  }

 protected:

  /**
   * Biquad filter
   */
  filter::Biquad<T> B;

  /**
   * @inheritdoc
   */
  T control(T e) {
    return B.step(e);
  }

};

/**
 * Proportional + Integral controller
 */
template<typename T>
class PI : public PID<T> {
 public:

  /**
   * Proportional-integral controller
   * @param Ts_ Time-step (s)
   * @param Kp_ Proportial gain
   * @param Ti_ Integrator time-constant (s)
   * @param Limit_ Maximum output value
   */
  explicit PI(T Ts_ = 1, T Kp_ = 1, T Ti_ = max<T>(), T Limit_ = max<T>()) : PID<T>(Ts_, Kp_, Ti_, 0, max<T>(),
                                                                           Limit_) {};
};

/**
 * Proportional + Derivative controller
 */
template<typename T>
class PD : public PID<T> {
 public:

  /**
   * Proportional-derivative controller
   * @param Ts_ Time-step (s)
   * @param Kp_ Proportial gain
   * @param Td Differentiator time-constant (s)
   * @param N Filter coefficicent
   * @param Limit_ Maximum output value
   */
  explicit PD(T Ts_ = 1, T Kp_ = 1, T Td_ = 0, T N_ = max<T>(), T Limit_ = max<T>()) : PID<T>(Ts_, Kp_, max<T>(), Td_,
                                                                                     N_, Limit_) {};
};

}

