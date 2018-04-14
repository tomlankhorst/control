/*
 * pid.h
 *
 * Classic P/PI/PID control
 *
 * @author Tom Lankhorst
 */

#ifndef CONTROL_CLASSIC_PID_H_
#define CONTROL_CLASSIC_PID_H_

#include <limits>
#include <algorithm>
#include "../filter/biquad.h"

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
	 * Proportional integral derivative controller
	 */
	template<typename T>
	class PID : public AbstractController<T>
	{
	public:
		PID(T Ts=1.0, T Kp=1.0, T Ti=max<T>(), T Td=0.0, T N=max<T>(), T Limit=max<T>()) : AbstractController<T>(Limit), B(
				(Kp*(4*Td/N + 2*Td*Ts/Ti/N + Ts*Ts/Ti + 4*Td + 2*Ts))/(4*Td/N + 2*Ts),
				-(Kp*(- Ts*Ts/Ti + 4*Td/N + 4*Td))/(2*Td/N + Ts),
				(Kp*(4*Td/N - 2*Td*Ts/Ti/N + Ts*Ts/Ti + 4*Td - 2*Ts))/(4*Td/N + 2*Ts),
				-(4*Td/N)/(2*Td/N + Ts),
				(2*Td/N - Ts)/(2*Td/N + Ts)
		) {};


	protected:

	  	/**
	  	 * Biquad filter
	  	 */
	  	filter::Biquad<T> B;

		/**
		 * @inheritdoc
		 */
		T control(T e)
		{
			return B.step(e);
		}

	};


	/**
	 * Proportional + Integral controller
	 */
	template<typename T>
	class PI : public PID<T> {
	public:
		PI(T Ts_=1, T Kp_=1, T Ti_=max<T>(), T Limit_=max<T>()) : PID<T>(Ts_, Kp_, Ti_, 0, max<T>(), Limit_) {};
	};

	/**
	 * Proportional + Derivative controller
	 */
	template<typename T>
	class PD : public PID<T> {
	public:
		PD(T Ts_=1, T Kp_=1, T Td_=0, T N_=max<T>(), T Limit_=max<T>()) : PID<T>(Ts_, Kp_, max<T>(), Td_, N_, Limit_) {};
	};

} }

#endif /* CONTROL_CLASSIC_PID_H_ */

