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

/**
 * State-space 
 *
 * Discretizes LTI (MIMO) state-space system.
 * Based upon Eigen matrix math lib. 
 * Uses fixed-size matrices and vectorization. 
 *
 * @tparam T storage-type
 * @tparam Nx number of states
 * @tparam Nu number of inputs
 * @tparam Ny number of outputs
 */
template<typename T, size_t Nx, size_t Nu=1, size_t Ny=1>
class ss {
public:
	using Tx = Eigen::Matrix<T, Nx, 1>;
	using Tu = Eigen::Matrix<T, Nu, 1>;
	using Ty = Eigen::Matrix<T, Ny, 1>;
	using TA = Eigen::Matrix<T, Nx, Nx>;
	using TB = Eigen::Matrix<T, Nx, Nu>;
	using TC = Eigen::Matrix<T, Ny, Nx>;
	using TD = Eigen::Matrix<T, Ny, Nu>;

private:
	const TA A;
  const TB B;
	const TC C;
	const TD D;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  /**
   * Construct a new SS and initialize the output and state to zero
   *
   * @param TA A state-transfer matrix
   * @param TB B input matrix
   * @param TC C output matrix
   * @param TD D feed-through matrix
   */
  ss(TA A, TB B, TC C, TD D) : A{A}, B{B}, C{C}, D{D} {
    x = Tx::Zero();
    y = Ty::Zero();
  }

  /**
   * @var Tx current state of the system
   */
	Tx x;

  /**
   * @var Ty current output of the system
   */
	Ty y;

  /**
   * Step the system
   *
   * @param Tu u input 
   * @return Ty output
   */
	Ty step(Tu u){
		x = A*x + B*u;
		y = C*x + D*u;
		return y;
	}
};

} }


#endif /* INCLUDE_CONTROL_SYSTEM_SS_H_ */
