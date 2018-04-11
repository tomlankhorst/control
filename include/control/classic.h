/*
 * classic.h
 *
 * Classic P/PI/PID control
 *
 * @author Tom Lankhorst
 */

#ifndef CONTROL_CLASSIC_H_
#define CONTROL_CLASSIC_H_

#include <limits>
#include <algorithm>

namespace control { namespace classic {

	/**
	 * Helper that returns either infinity or maximum value
	 */
	template <typename T>
	constexpr T max(){
		return std::numeric_limits<T>::has_infinity
				? std::numeric_limits<T>::infinity() : std::numeric_limits<T>::max();
	}

	template <typename T>
	class AbstractController {
		static_assert(std::numeric_limits<T>::is_signed, "Signed type required");
	public:
		AbstractController(T Limit_=max<T>()) : Limit(Limit_) {};
		virtual ~AbstractController() {};

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
		T step(T e)
		{
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
		void setLimit(T limit)
		{
			Limit = limit;
		}

	protected:

		/**
		 * Limit the output
		 *
		 * @param T u
		 * @return T
		 */
		T clip(T u)
		{
			if(Limit==std::numeric_limits<T>::infinity()){
				return u;
			}

			// Determine if the signal is in range
			if( !clipping && (u > Limit || u < -Limit) ){
				clipping = true;
			} else if ( clipping && ( u >= -Limit && u <= Limit) ) {
				clipping = false;
			}

			// If limiting, then clip to min / max
			if( clipping ) {
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
	template <typename T>
	class P : public AbstractController<T> {
	public:
		P(T Kp_=1.0, T Limit_=max<T>()) : AbstractController<T>(Limit_), Kp(Kp_) {};
		~P() {};

		// Proportional gain
		const T Kp;

	protected:
		/**
		 * @inheritdoc
		 */
		T control(T e)
		{
			// The output of the controller
			T u;

			// Proportional gain
			u = Kp*e;

			return u;
		};
	};

	/**
	 * Proportional + Integral controller
	 */
	template<typename T>
	class PI : public P<T> {
	public:
		PI(T Ts_=1.0, T Kp_=1.0, T Ti_=max<T>(), T Limit_=max<T>())
			: P<T>(Kp_, Limit_), Ti(Ti_), Ts(Ts_) {};
		~PI() {};

	protected:
		// Integral time constant
		const T Ti;

		// Time-step
		const T Ts;

		// Integral error
		T e_int = 0;

		/**
		 * @inheritdoc
		 */
		T control(T e)
		{
			T u;

			// Get the proportional control
			u = P<T>::control(e);

			// Add up the integral part to the error
			u+= IF(e)/Ti;

			return u;
		}

		/**
		 * (I) integrator
		 *
		 * Does only integrate when not clipping
		 *
		 * @param T error
		 * @return T integral
		 */
		T IF(T e)
		{
			// Integrate the error when not clipping
			if( !AbstractController<T>::clipping ) {
				e_int += e*Ts;
			}
			return e_int;
		}
	};

	template<typename T>
	class PID : public PI<T>
	{
	public:
		PID(T Ts_=1.0, T Kp_=1.0, T Ti_=max<T>(), T Td_=0.0, T N_=max<T>(), T Limit_=max<T>())
			: PI<T>(Kp_, Ti_, Ts_, Limit_), Td(Td_), N(N_) {};
		~PID() {};

	protected:
		// Derivative time constant
		const T Td;

		// Filtering constant
		const T N;

		/**
		 * @inheritdoc
		 */
		T control(T e)
		{
			T u;

			// Get the PI control
			u = PI<T>::control(e);

			// Add up the derivative part
			u+= Td/(Td/N+DF(e))*P<T>::control(e);

			return u;
		}

		/**
		 * (D) integrator
		 *
		 * Execute AFTER IF
		 *
		 * @param T error
		 * @return T integral
		 */
		T DF(T e)
		{
			// Same value as (I) integrator
			return PI<T>::e_int;
		}
	};

} }

#endif /* CONTROL_CLASSIC_H_ */

