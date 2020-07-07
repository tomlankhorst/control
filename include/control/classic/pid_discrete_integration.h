#include <iostream>
#include <type_traits>

namespace control::classic {

#include <iostream>

#include <type_traits>

template<typename T>
class basic_pid {
    static_assert(std::is_floating_point_v<T>);

public:
    enum class IntegratorFormula {
        // Best when Nyquist limit >> controller bandwidth. Might yield instability. 
        ForwardEuler,   
        // Always yields a stable DT system when discretizing a stable CT system. 
        BackwardEuler,  
        // Always yields a stable DT system when discretizing a stable CT system. 
        // Best match of frequency-domain properties between CT and DT system. 
        Trapezoidal,    
    };

    struct config {
        // Proportional, Integral, Derivate gain
        const T Kp = 1, Ki = 0, Kd = 0;
        // Filter time-step
        const T Tf = 0;
        // Timestep
        const T Ts = 1;
        // Discrete integrator formula
        const IntegratorFormula integrator_formula = IntegratorFormula::Trapezoidal;
    };

    explicit constexpr basic_pid (const config& c) : 
        coeff_ { calculate_coeff(c) } {}

    constexpr T operator()(T u) noexcept {
        T y = 0;

        /* Direct form II transposed */
        y = u * coeff_.b[0] + state_.w[0];
        state_.w[0] = u * coeff_.b[1] - coeff_.a[0] * y + state_.w[1];
        state_.w[1] = u * coeff_.b[2] - coeff_.a[1] * y;

        return y;
    }

private:
    struct coeff_t {
        T b[3];
        T a[2];
    };
    struct state_t {
        T w[2] = {0, 0};
    };

    static constexpr coeff_t calculate_coeff(const config& c) {
        switch (c.integrator_formula) {
            case IntegratorFormula::BackwardEuler:
                return backward_euler_coeff(c);
            case IntegratorFormula::ForwardEuler:
                return forward_euler_coeff(c);
            case IntegratorFormula::Trapezoidal:
            default:
                return trapezoidal_coeff(c);
        }
    }

    static constexpr coeff_t forward_euler_coeff(const config& c) {
        const auto& [Kp, Ki, Kd, Tf_, Ts, _] = c;
        // ForwardEuler: Kp + Kd/(Tf + Ts/(z - 1)) + (Ki*Ts)/(z - 1)
        const auto Tf = Kd ? Tf_ : 1; // If Kd not set, set Tf to 1 for stability
        return {{
            (Kd + Kp*Tf)/Tf,
            (Kp*Ts - 2*Kp*Tf - 2*Kd + Ki*Tf*Ts)/Tf,
            (Kd + Ki*Ts*Ts + Kp*Tf - Kp*Ts - Ki*Tf*Ts)/Tf,
        }, {
            (Ts - 2*Tf)/Tf,
            (Tf - Ts)/Tf,
        }};
    }

    static constexpr coeff_t backward_euler_coeff(const config& c) {
        const auto& [Kp, Ki, Kd, Tf, Ts, _] = c;
        // BackwardEuler: Kp + Kd/(Tf + (Ts*z)/(z - 1)) + (Ki*Ts*z)/(z - 1)
        return {{
            (Kd + Ki*Ts*Ts + Kp*Tf + Kp*Ts + Ki*Tf*Ts)/(Tf + Ts),
            -(2*Kd + 2*Kp*Tf + Kp*Ts + Ki*Tf*Ts)/(Tf + Ts),
            (Kd + Kp*Tf)/(Tf + Ts),
        }, {
            -(2*Tf + Ts)/(Tf + Ts),
            Tf/(Tf + Ts),
        }};
    }

    static constexpr coeff_t trapezoidal_coeff(const config& c) {
        const auto& [Kp, Ki, Kd, Tf, Ts, _] = c;
        // Trapezoidal: Kp + Kd/(Tf + (Ts*(z + 1))/(2*(z - 1))) + (Ki*Ts*(z + 1))/(2*(z - 1))
        return  {{
	        (4*Kd + Ki*Ts*Ts + 4*Kp*Tf + 2*Kp*Ts + 2*Ki*Tf*Ts)/(4*Tf + 2*Ts),
	        -(4*Kd - Ki*Ts*Ts + 4*Kp*Tf)/(2*Tf + Ts),
	        (4*Kd + Ki*Ts*Ts + 4*Kp*Tf - 2*Kp*Ts - 2*Ki*Tf*Ts)/(4*Tf + 2*Ts),
        }, {
	        -(4*Tf)/(2*Tf + Ts),
	        (2*Tf - Ts)/(2*Tf + Ts),
        }};
    }

    const coeff_t coeff_;
    state_t state_;
};

using pid = basic_pid<double>;

}
