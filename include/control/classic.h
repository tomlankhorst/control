/*
 * classic.h
 *
 * Classic P/PI/PID control
 *
 * @author Tom Lankhorst
 */

#ifndef CONTROL_CLASSIC_H_
#define CONTROL_CLASSIC_H_

namespace control { namespace classic {

	template <typename T>
	class AbstractController {
	public:

		AbstractController(T Limit_ = 1.0) : Limit(Limit_) {};
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
		 * Limit the output
		 *
		 * @param T u
		 * @return T
		 */
		T clip(T u)
		{
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
		P(T Kp_, T Limit_=1.0) : AbstractController<T>(Limit_), Kp(Kp_) {};
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
		PI(T Ts_, T Kp_=1.0, T Ti_=std::numeric_limits<T>::infinity(), T Limit_=1.0) : P<T>(Kp_, Limit_), Ti(Ti_), Ts(Ts_) {};
		~PI() {};

	protected:
		// Integral time constant
		const T Ti;

		// Time-step
		const T Ts;

		// Integral error
		T e_int = 0.0;

		/**
		 * @inheritdoc
		 */
		T control(T e)
		{
			T u;

			// Get the proportional control
			u = P<T>::control(e);

			// Add up the integral part to the error
			u+= (1/Ti)*IF(e);

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
			if( !clipping ) {
				e_int += e*Ts;
			}
			return e_int;
		}
	};

	template<typename T>
	class PID : public PI<T>
	{
	public:
		PID(T Ts_, T Kp_=1, T Ti_=std::numeric_limits<T>::infinity(), T Td_=0.0, T N_=std::numeric_limits<T>::infinity(), T Limit_=1.0)
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
			return e_int;
		}
	};

} }

#endif /* CONTROL_CLASSIC_H_ */
