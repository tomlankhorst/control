/*
 * ss.h
 *
 *  Created on: Apr 29, 2018
 *      Author: tomlankhorst
 */

#ifndef INCLUDE_CONTROL_SYSTEM_SS_H_
#define INCLUDE_CONTROL_SYSTEM_SS_H_

#include <Eigen/Dense>

namespace control { namespace system {

template<typename T, size_t Nx, size_t Nu=1, size_t Ny=1>
class ss {
public:
	using Tx = Eigen::Matrix<T, Nx, 1>;
	using Tu = Eigen::Matrix<T, Nu, 1>;
	using Ty = Eigen::Matrix<T, Ny, 1>;
	using TA = Eigen::Matrix<T, Nx, Nx>;
	using TB = Eigen::Matrix<T, Nx, Nu>;
	using TC = Eigen::Matrix<T, Ny, Nx>;
	using TD = Eigen::Matrix<T, Ny, Ny>;

private:
	const TA A;
  const TB B;
	const TC C;
	const TD D;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ss(TA A, TB B, TC C, TD D) : A{A}, B{B}, C{C}, D{D} {
    x = Tx::Zero();
    y = Ty::Zero();
  }

	Tx x;
	Ty y;

	Ty step(Tu u){
		x = A*x + B*u;
		y = C*x + D*u;
		return y;
	}
};

} }


#endif /* INCLUDE_CONTROL_SYSTEM_SS_H_ */
