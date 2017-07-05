/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*    Authors: Meloun Martin <meloumar@cmp.felk.cvut.cz>
*             Tomas Pajdla  <pajdla@cmp.felk.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#ifndef EXT_GENERAL6R_KIN_H
#define EXT_GENERAL6R_KIN_H

#include "ext_general6r_robot.h"

/* Limits (described in structure) */
#define LIM_DEFAULT_PERTURBATION                     1e-4
#define LIM_DEFAULT_IMAGINARY_TOL                    0.010
#define LIM_DEFAULT_ROOT_TOL                         0.010
#define LIM_DEFAULT_EIGENVALUE_SAFETY                0.100
#define LIM_DEFAULT_EIGENVALUE_INFINITE_LIMIT        1e-6
#define LIM_DEFAULT_OPTIMIZATION_TOL                 1e-6
#define LIM_DEFAULT_OPTIMIZATION_MAX_STEPS           4
#define LIM_DEFAULT_OPTIMIZATION_BIGDIFF_TOL         1e-4
#define LIM_DEFAULT_OPTIMIZATION_BIGDIFF_EXTRA_STEPS 12
#define LIM_DEFAULT_OPTIMIZATION_RECALC_LIMIT        DEG(1.1)
#define LIM_DEFAULT_DULPICATE_SOLUTION_TOL           DEG(0.5)
#define LIM_DEFAULT_TEST_JOINT_LIMITS                1

/* Calculation flags */

/* 0000xxxx - whether solution was calculated with alt. method on joints 4 and 5 */
#define FLAG1_SOL_N_ALTERNATE_METHOD(n)       (1 << (n-1))
/* xxxx0000 - whether solution was being optimized on Z*p = 0 equation */
#define FLAG1_SOL_N_OPTIMIZED_ON_Z(n)         (1 << (16+(n-1)))

/* 0000xxxx - whether solution is singular on some joint */
#define FLAG2_SOL_N_SINGULAR(n)               (1 << (n-1))
/* 00010000 - whether martix N rank was not eight */
#define FLAG2_N_MATRIX_RANK_NOT_EIGHT         0x00010000

/* Size of flags */
#define FLAGS_SIZE                            2

/* Limits structure */
typedef struct
{
	/* Specifies perturbation tolerance for sigular statesspecifies perturbation tolerance for sigular states
	 * Defaults 1e-4
	 */
	double perturbation;

	/* Specifies tolerance for deformed multiple roots solutions in which a complex conjugate pair counts as double real root
	 * Defautls to 0.01
	 */
	double imaginary_tol;

	/* After filtering with imagtol, solutions within this distance are considered as a multiple root solution and thus marked so,
	 * to denote possible geometric multiplicity being greater than one
	 * Defaults to 0.05
	 */
	double root_tol;

	/* Safety check to validate eigenvector values when solving x4 and s5, c5 - if it is not passed, then the solution is checked,
	 * again as if the geometric multiplicity was greater than one.
	 * Defaults to 0.1
	 */
	double eigenvalue_safety;

	/* When last element of the eigenvector is below this value, consider x4 being infinity.
	 * Defaults to 1e-6
	 */
	double eigenvalue_infinite_limit;

	/* Tolerance for Newton optimization over j3, j4, j5
	 * Defaults to 1e-6
	 */
	double optimization_tol;

	/* Maximum amount of steps for Newton optimization
	 * Defaults to 4
	 */
	int optimization_max_steps;

	/* Tolerance for Newton optimization over j3, j4, j5 to consider the difference as big
	 * and allow more steps
	 * Defaults to 1e-4
	 */
	double optimization_bigdiff_tol;

	/* Maximum amount of extra steps for Newton optimization for big difference
	 * Defaults to 12
	 */
	int optimization_bigdiff_extra_steps;

	/* Limit after which sin and cos is recalculated properly, instead of being approximated
	 * Defaults to 1.1 degree (1.1*pi/180), which results in 1e-6 difference between real and approximated result)
	 */
	double optimization_recalc_limit;

	/* Tolerance used in removing duplicate solutions
	 * Defaults to 1 degree (pi/180)
	 */
	double dulpicate_solution_tol;

	/* Whether to remove solutions that are not within joint limits.
	 * Defaults to 1 (YES)
	 */
	int test_joint_limits;
} gen6rikt_limits_t;

/* Default limits */
extern gen6rikt_limits_t default_limits;

/* General 6R Direct kinematics task */
void general_6r_dkt(double* MhV, gen6r_robot_t* robot, double* J);

/* General 6R Inverse kinematics task */
void general_6r_ikt(double* J, int* no_solutions, gen6r_robot_t* robot, double* MhV, gen6rikt_limits_t* robot_limits, unsigned int* flags);

#endif //!EXT_GENERAL6R_KIN_H
