/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*    Authors: Meloun Martin <meloumar@cmp.felk.cvut.cz>
*             Tomas Pajdla  <pajdla@cmp.felk.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#include "ext_general6r_robot.h"
#include "ext_general6r_kin.h"
#include "ext_general6r_opwrap.h"

/* Internal states for sin / cos equations */
#define SIN_COS_FULL_RANK   0
#define SIN_COS_FIXED_COS   1
#define SIN_COS_FIXED_SIN   2
#define SIN_COS_ZERO_RANK   3
#define SIN_COS_NO_SOLUTION 4

#define SIN_COS_UNOPTIMIZED_TOLERANCE 0.5

/* Other */
#define EIG_VALUE_SKIP                               -1
#define EIG_VALUE_ALTERNATE_METHOD                   -2

/* Solution structure for joints 3, 4, 5 */
typedef struct
{
	double s3, c3, j3;
	double s4, c4, j4;
	double s5, c5, j5;

} sol345_t;

/* Create default limits structure */
gen6rikt_limits_t default_limits =
{
	/*.perturbation                     =*/ LIM_DEFAULT_PERTURBATION,
	/*.imaginary_tol                    =*/ LIM_DEFAULT_IMAGINARY_TOL,
	/*.root_tol                         =*/ LIM_DEFAULT_ROOT_TOL,
	/*.eigenvalue_safety                =*/ LIM_DEFAULT_EIGENVALUE_SAFETY,
	/*.eigenvalue_infinite_limit        =*/ LIM_DEFAULT_EIGENVALUE_INFINITE_LIMIT,
	/*.optimization_tol                 =*/ LIM_DEFAULT_OPTIMIZATION_TOL,
	/*.optimization_max_steps           =*/ LIM_DEFAULT_OPTIMIZATION_MAX_STEPS,
	/*.optimization_bigdiff_tol         =*/ LIM_DEFAULT_OPTIMIZATION_BIGDIFF_TOL,
	/*.optimization_bigdiff_extra_steps =*/ LIM_DEFAULT_OPTIMIZATION_BIGDIFF_EXTRA_STEPS,
	/*.optimization_recalc_limit        =*/ LIM_DEFAULT_OPTIMIZATION_RECALC_LIMIT,
	/*.dulpicate_solution_tol           =*/ LIM_DEFAULT_DULPICATE_SOLUTION_TOL,
	/*.test_joint_limits                =*/ LIM_DEFAULT_TEST_JOINT_LIMITS,
};

/* Convert x = tan(j/2) into sin(j), cos(j) and j */
void x2scj(double* s, double* c, double* j, double x)
{
	double d = 1/(1+x*x);

	if (fabs(d) < M_BIGEPS)
	{
		*s = 0;
		*c = -1;
		*j = M_PI;
	}
	else
	{
		*s = 2*x*d;
		*c = (1-x*x)*d;
		*j = 2*atan(x);
	}
}

/* Newton optimization over Z * p = 0, returns no. steps of opt */
int newton_optimization_345(double* Za, double* Zb, double* Zc, sol345_t* sol, gen6rikt_limits_t* limits)
{
	/* diff storage */
	sol345_t cur;
	double lb, ub;
	double y[6*1], p[9*1], dp4[9*1], dp5[9*1], dy[6*3];
	double z[6*9], zd[6*9], d[3], yy[3];
	int i, opt, rnk, steps, bigdiff_steps, optsteps;
	int piv[6*1];

	/* Init */
	cur.j3 = 0;
	cur.j4 = 0;
	cur.j5 = 0;

	lb = -limits->optimization_recalc_limit;
	ub = limits->optimization_recalc_limit;

	bigdiff_steps = limits->optimization_bigdiff_extra_steps;
	optsteps = 0;

	for(steps = limits->optimization_max_steps; steps > 0; steps--)
	{
		cur.s3 = sol->s3 * (1 - (cur.j3*cur.j3)/2) + sol->c3 * cur.j3;
		cur.c3 = sol->c3 * (1 - (cur.j3*cur.j3)/2) - sol->s3 * cur.j3;

		cur.s4 = sol->s4 * (1 - (cur.j4*cur.j4)/2) + sol->c4 * cur.j4;
		cur.c4 = sol->c4 * (1 - (cur.j4*cur.j4)/2) - sol->s4 * cur.j4;

		cur.s5 = sol->s5 * (1 - (cur.j5*cur.j5)/2) + sol->c5 * cur.j5;
		cur.c5 = sol->c5 * (1 - (cur.j5*cur.j5)/2) - sol->s5 * cur.j5;

		/* P vector */
		p[0] = cur.s4 * cur.s5;
		p[1] = cur.s4 * cur.c5;
		p[2] = cur.c4 * cur.s5;
		p[3] = cur.c4 * cur.c5;
		p[4] = cur.s4;
		p[5] = cur.c4;
		p[6] = cur.s5;
		p[7] = cur.c5;
		p[8] = 1.0;

		/* Calculate difference */
		for (i = 0; i < ARRAY_SIZE(z); i++)
			z[i] = (Za[i] * cur.s3) + (Zb[i] * cur.c3) + Zc[i];

		/* Calculate y */
		matrix_mult(y, z, 6, 9, p, 9, 1);

		/* Check if we need to optimize futher */
		opt = 0;
		for (i = 0; i < ARRAY_SIZE(y); i++)
		{
			if (fabs(y[i]) > limits->optimization_tol)
			{
				if (bigdiff_steps > 0 && fabs(y[i]) > limits->optimization_bigdiff_tol)
				{
					bigdiff_steps--;
					steps++;
				}

				opt = 1;
				break;
			}
		}

		if (!opt)
			break;

		/* P vector, j4 derivation */
		dp4[0] = cur.c4 * cur.s5;
		dp4[1] = cur.c4 * cur.c5;
		dp4[2] = -cur.s4 * cur.s5;
		dp4[3] = -cur.s4 * cur.c5;
		dp4[4] = cur.c4;
		dp4[5] = -cur.s4;
		dp4[6] = 0;
		dp4[7] = 0;
		dp4[8] = 0;

		/* P vector, j5 derivation */
		dp5[0] = cur.s4 * cur.c5;
		dp5[1] = cur.s4 * -cur.s5;
		dp5[2] = cur.c4 * cur.c5;
		dp5[3] = cur.c4 * -cur.s5;
		dp5[4] = 0;
		dp5[5] = 0;
		dp5[6] = cur.c5;
		dp5[7] = -cur.s5;
		dp5[8] = 0;

		/* dy, first column */
		for (i = 0 ; i < ARRAY_SIZE(z); i++)
			zd[i] = (Za[i] * cur.c3) - (Zb[i] * cur.s3);

		/* Leading dimension is the same as dimension */
		matrix_mult(&(dy[_2D_IDX(1,1,6,3)]), zd, 6, 9, p, 9, 1);
		matrix_mult(&(dy[_2D_IDX(1,2,6,3)]), z, 6, 9, dp4, 9, 1);
		matrix_mult(&(dy[_2D_IDX(1,3,6,3)]), z, 6, 9, dp5, 9, 1);

		/* Now, decompose dy with LU decomposition and determine rank */
		if (lu_quick(dy, piv, 6, 3))
			break; /* Error, quit! */

		rnk = 0;

		for (i = 0; i < 3; i++)
		{
			if (fabs(dy[_2D_IDX(i,i,6,3)]) > limits->perturbation)
				rnk++;
		}

		/* No convergency - quit */
		if (rnk == 0)
			break;

		/* Obtain the differential */
		yy[0] = y[piv[0]];
		yy[1] = y[piv[1]] - (yy[0] * dy[_2D_IDX(2,1,6,3)]);
		yy[2] = y[piv[2]] - (yy[0] * dy[_2D_IDX(3,1,6,3)]) - (yy[1] * dy[_2D_IDX(3,2,6,3)]);

		if (fabs(dy[_2D_IDX(3,3,6,3)]) > limits->perturbation)
			d[2] = yy[2] / dy[_2D_IDX(3,3,6,3)];
		else
			d[2] = 0;

		if (fabs(dy[_2D_IDX(2,2,6,3)]) > limits->perturbation)
			d[1] = (yy[1] - (d[2] * dy[_2D_IDX(2,3,6,3)])) / dy[_2D_IDX(2,2,6,3)];
		else
			d[1] = 0;

		if (fabs(dy[_2D_IDX(1,1,6,3)]) > limits->perturbation)
			d[0] = (yy[0] - (d[2] * dy[_2D_IDX(1,3,6,3)]) - (d[1] * dy[_2D_IDX(1,2,6,3)])) / dy[_2D_IDX(1,1,6,3)];
		else
			d[0] = 0;

		/* Subtract the differential and check bounds, and recalculate the sin / cos if needed */
		cur.j3 -= d[0];
		cur.j4 -= d[1];
		cur.j5 -= d[2];

		if (cur.j3 < lb || cur.j3 > ub)
		{
			sol->j3 += cur.j3;
			sol->s3 = sin(sol->j3);
			sol->c3 = cos(sol->j3);
			cur.j3 = 0;
		}

		if (cur.j4 < lb || cur.j4 > ub)
		{
			sol->j4 += cur.j4;
			sol->s4 = sin(sol->j4);
			sol->c4 = cos(sol->j4);
			cur.j4 = 0;
		}

		if (cur.j5 < lb || cur.j5 > ub)
		{
			sol->j5 += cur.j5;
			sol->s5 = sin(sol->j5);
			sol->c5 = cos(sol->j5);
			cur.j5 = 0;
		}

		optsteps++;
	}

	/* Write back to the solution */
	if (i > 0)
	{
		sol->j3 += cur.j3;
		sol->j4 += cur.j4;
		sol->j5 += cur.j5;

		sol->s3 = sin(sol->j3);
		sol->c3 = cos(sol->j3);

		sol->s4 = sin(sol->j4);
		sol->c4 = cos(sol->j4);

		sol->s5 = sin(sol->j5);
		sol->c5 = cos(sol->j5);
	}

	return optsteps;
}

/* Ensure that sin(j) and cos(j) are valid: (sin(j)^2 + cos(j)^2 - 1) < tol */
int sin_cos_valid(double s, double c, double tol)
{
	return fabs(1 - (s*s + c*c)) < tol;
}

/* Check if we're within joint limits */
int is_within_limit(gen6r_robot_t* robot, int joint, double theta)
{
	return theta >= robot->joint_limits[JOINT_MIN(joint)] && theta <= robot->joint_limits[JOINT_MAX(joint)];
}

void sin_cos_equation_triangle(double* s, double *c, double* j, int* f,
															 const double* R, const double* B, const gen6rikt_limits_t* limits,
															 int after_optimization, int allow_multiple_solutions)
{
	/* FIXME: The non-opt tolerance is hardcoded */
	double tolerance = after_optimization ? limits->perturbation : SIN_COS_UNOPTIMIZED_TOLERANCE;
	double X[2] = { 0, 0 };
	int rank = 0, x0 = 0;

	/* First, let's check the rank of the matrix */
	if (fabs(R[_2D_IDX(2,2,2,2)]) > limits->perturbation)
	{
		X[1] = B[1] / R[_2D_IDX(2,2,2,2)];
		rank++;
	}

	if (fabs(R[_2D_IDX(1,1,2,2)]) > limits->perturbation)
	{
		X[0] = (B[0] - R[_2D_IDX(1,2,2,2)]*X[1]) / R[_2D_IDX(1,1,2,2)];
		x0 = 1;
		rank++;
	}

	/* Now, we return all possible solutions based on rank */
	switch (rank)
	{
		case 2:
			/* FULL RANK - 1 solution */
			if (sin_cos_valid(X[0], X[1], tolerance))
			{
				*j = atan2(X[0], X[1]);

				if (after_optimization)
				{
					*s = X[0];
					*c = X[1];
				}
				else
				{
					*s = sin(*j);
					*c = cos(*j);
				}

				*f = SIN_COS_FULL_RANK;
			}
			else
			{
				/* Still pass the sin / cos values for verbose */
				*s = X[0];
				*c = X[1];
				*f = SIN_COS_NO_SOLUTION;
			}
			return;

		case 1:
			/* LOW RANK - 2 solutions, becaus sin and cos are bounded to each other */
			if (!allow_multiple_solutions)
			{
				/* Still pass the sin / cos values for verbose */
				*s = X[0];
				*c = X[1];
				*f = SIN_COS_NO_SOLUTION;
				return;
			}

			/* Check whether sin or cos value is known with sign */
			if (!x0)
			{
				/* cos is known, check if it's valid */
				if (1 - fabs(X[1]) < -tolerance)
				{
					/* Still pass the sin / cos values for verbose */
					*s = X[0];
					*c = X[1];
					*f = SIN_COS_NO_SOLUTION;
					return;
				}

				if (fabs(X[1]) > 1)
					X[1] = SGN(X[1]);

				*c = X[1];
				*j = acos(*c);
				*s = sin(*j); /* +/- */
				*f = SIN_COS_FIXED_COS;
				return;
			}
			else
			{
				/* sin is known, check if it's valid */
				if (1 - fabs(X[0]) < -tolerance)
				{
					/* Still pass the sin / cos values for verbose */
					*s = X[0];
					*c = X[1];
					*f = SIN_COS_NO_SOLUTION;
					return;
				}

				if (fabs(X[0]) > 1)
					X[0] = SGN(X[0]);

				*s = X[0];
				*j = asin(*s);
				*c = cos(*j); /* +/- */
				*f = SIN_COS_FIXED_SIN;
				return;
			}

		case 0:
			/* ZERO RANK - infinite solutions */
			if (!allow_multiple_solutions || fabs(B[0]) > limits->perturbation)
			{
				/* Still pass the sin / cos values for verbose */
				*s = X[0];
				*c = X[1];
				*f = SIN_COS_NO_SOLUTION;
				return;
			}
			else
			{
				*s = 0;
				*c = 1;
				*j = 0;
				*f = SIN_COS_ZERO_RANK;
				return;
			}
	}
}

void general_6r_ikt(double* J, int* no_solutions, gen6r_robot_t* robot, double* MhV, gen6rikt_limits_t* limits, unsigned int* flags)
{
	int i, j, k, l, m, srank;
	int sol_newton_optimized, sol_alternate_method;

	/* N, P matrices */
	double N[14*8];
	double Pa[14*9], Pb[14*9], Pc[14*9];

	/* QR of N */
	double Q[14*14], R[14*8];

	/* Z matrices (Qt*Pa, Qt*Pb, Qt*Pc) */
	double Za[6*9], Zb[6*9], Zc[6*9];

	/* ME matrices to solve s1,c1 and s2,c2 */
	double MEa[4*9], MEb[4*9], MEc[4*9];

	/* Currently used Me matrix for calculation */
	double MECalc[2*9], LP[9*1], LB1[2*1], LB2[2*1], RB1[2*2], RB2[2*2];

	/* Generalized eig. problem for x3 matrices */
	double GeM1x3[24*24], GeM2x3[24*24];

	/* Generalized eigenvectors and eigenvalues */
	double alpha_r[24], alpha_i[24], beta[24];
	double eigvector[24*24];
	double eigvalue[16];

	/* Index for valid eigenvalues */
	int valid_eigenval_index[16];
	int no_eigvalues;

	GENERAL_6R_DEBUG("START\n", 0);

	/* Reset number of solutions */
	*no_solutions = 0;

	/* Reset flags */
	if (flags)
	{
		flags[0] = 0;
		flags[1] = 0;
	}

	/* Fill matrices P and Q */
	memset(Pa, 0, sizeof(Pa));
	memset(Pb, 0, sizeof(Pb));
	memset(Pc, 0, sizeof(Pc));

	memset(N, 0, sizeof(N));

	/* ======================================================================
	 * STEP 1: Compute matrices Za, Zb, Zc
	 * ====================================================================*/

	robot->form_base_ikt_matrices(robot, N, Pa, Pb, Pc, MhV);

	/* Calculate QR of N */
	memset(Q,  0, sizeof(Q));
	memset(R,  0, sizeof(R));
	GENERAL_6R_DEBUG("Decomposing N matrix using QR.\n");
	if (qr(Q, R, N, 14, 8))
	{
		GENERAL_6R_DEBUG("Failed decomposing N matrix using QR.\n");
		no_solutions = 0;
		return;
	}
	GENERAL_6R_DEBUG("Decomposed N matrix using QR.\n");

	/* Subtract R matrix for calculating j1, j2 later */
	RB1[_2D_IDX(1,1,2,2)] = R[_2D_IDX(5,5,14,8)];
	RB1[_2D_IDX(1,2,2,2)] = R[_2D_IDX(5,6,14,8)];
	RB1[_2D_IDX(2,1,2,2)] = R[_2D_IDX(6,5,14,8)];
	RB1[_2D_IDX(2,2,2,2)] = R[_2D_IDX(6,6,14,8)];

	RB2[_2D_IDX(1,1,2,2)] = R[_2D_IDX(7,7,14,8)];
	RB2[_2D_IDX(1,2,2,2)] = R[_2D_IDX(7,8,14,8)];
	RB2[_2D_IDX(2,1,2,2)] = R[_2D_IDX(8,7,14,8)];
	RB2[_2D_IDX(2,2,2,2)] = R[_2D_IDX(8,8,14,8)];

	/* Calculate rank and select equations */
	srank = 8;
	for (i = 1; i <= 8; i++)
	{
		if (fabs(R[_2D_IDX(i, i, 14, 8)]) < limits->perturbation)
		{
			srank = -1;
			break;
		}
	}

	/* It's not full rank, we have to do SVD */
	if (srank < 0)
	{
		/* SVD of R */
		double Ut[14*14], U8[8*8], S8[8] , R8[8*8];

		/* U^T*Q^T */
		double Qt[14*14];

		/* For ME */
		double* Qe;

		/* Select R8 (not using R, because svd would change it) */
		for (i = 1; i <= 8; i++)
			for (j = 1; j <= 8; j++)
				R8[_2D_IDX(i,j,8,8)] = R[_2D_IDX(i,j,14,8)];

		/* Calculate SVD */
		memset(Ut,  0, sizeof(Ut));
		memset(U8,  0, sizeof(U8));
		memset(S8,  0, sizeof(S8));

		GENERAL_6R_DEBUG("N rank is not full, decomposing futher with SVD.\n");
		if (svd(U8, S8, NULL, R8, 8, 8))
		{
			GENERAL_6R_DEBUG("Failed decomposing futher with SVD.\n");
			no_solutions = 0;
			return;
		}
		GENERAL_6R_DEBUG("Decomposed futher with SVD.\n");

		/* Calculate rank */
		srank = 0;
		do
		{
			srank++;
		} while (S8[srank] > limits->perturbation && srank < 8);

		/* Expand U and transpose it, we need to fill it with ones on the
		 * remaining diagonal elements, so can't use leading dimension
		 */
		for (i = 1; i <= 8; i++)
			for (j = 1; j <= 8; j++)
				Ut[_2D_IDX_T(i,j,14,14)] = U8[_2D_IDX(i,j,8,8)];

		for (i = 9; i <= 14; i++)
			Ut[_2D_IDX(i,i,14,14)] = 1;

		/* Select first interesting Q matrix entry */
		Qe = &(Q[_2D_IDX_T(5,1,14,14)]);

		/* Calculate ME */
		matrix_mult_ld(MEa, Qe, "T", 14, 4, 14, Pa, "N", 14, 14, 9);
		matrix_mult_ld(MEb, Qe, "T", 14, 4, 14, Pb, "N", 14, 14, 9);
		matrix_mult_ld(MEc, Qe, "T", 14, 4, 14, Pc, "N", 14, 14, 9);

		/* Calculate Qt */
		matrix_mult_ex(Qt, Ut, "N", 14, 14, Q, "T", 14, 14);
		{
			/* Select first interesting Q matrix entry */
			double* Qe = &(Qt[_2D_IDX(9-(8-srank),1,14,14)]);

			/* Calculate Z */
			matrix_mult_ld(Za, Qe, "N", 14, 6, 14, Pa, "N", 14, 14, 9);
			matrix_mult_ld(Zb, Qe, "N", 14, 6, 14, Pb, "N", 14, 14, 9);
			matrix_mult_ld(Zc, Qe, "N", 14, 6, 14, Pc, "N", 14, 14, 9);
		}

		/* update flags */
		if (flags)
			flags[1] |= FLAG2_N_MATRIX_RANK_NOT_EIGHT;
	}
	else
	{
		/* Select first interesting Q matrix entry for Z matrix */
		double* Qe = &(Q[_2D_IDX_T(9,1,14,14)]);

		/* Calculate Z */
		matrix_mult_ld(Za, Qe, "T", 14, 6, 14, Pa, "N", 14, 14, 9);
		matrix_mult_ld(Zb, Qe, "T", 14, 6, 14, Pb, "N", 14, 14, 9);
		matrix_mult_ld(Zc, Qe, "T", 14, 6, 14, Pc, "N", 14, 14, 9);

		/* Select first interesting Q matrix entry */
		Qe = &(Q[_2D_IDX_T(5,1,14,14)]);

		/* Calculate ME */
		matrix_mult_ld(MEa, Qe, "T", 14, 4, 14, Pa, "N", 14, 14, 9);
		matrix_mult_ld(MEb, Qe, "T", 14, 4, 14, Pb, "N", 14, 14, 9);
		matrix_mult_ld(MEc, Qe, "T", 14, 4, 14, Pc, "N", 14, 14, 9);
	}

	/* Debugging info */
	GENERAL_6R_DEBUG("Matrix N has rank %d.\n", srank);

	/* ======================================================================
	 * STEP 2: Solve dyalitic polynomial equation
	 * ====================================================================*/

	/* Form the two matrices for generalized eigenvalue problem */
	memset(GeM1x3, 0, sizeof(GeM1x3));
	memset(GeM2x3, 0, sizeof(GeM2x3));

	GeM1x3[_2D_IDX(1,1,24,24)] = 1;
	GeM1x3[_2D_IDX(2,2,24,24)] = 1;
	GeM1x3[_2D_IDX(3,3,24,24)] = 1;
	GeM1x3[_2D_IDX(4,4,24,24)] = 1;
	GeM1x3[_2D_IDX(5,5,24,24)] = 1;
	GeM1x3[_2D_IDX(6,6,24,24)] = 1;
	GeM1x3[_2D_IDX(7,7,24,24)] = 1;
	GeM1x3[_2D_IDX(8,8,24,24)] = 1;
	GeM1x3[_2D_IDX(9,9,24,24)] = 1;
	GeM1x3[_2D_IDX(10,10,24,24)] = 1;
	GeM1x3[_2D_IDX(11,11,24,24)] = 1;
	GeM1x3[_2D_IDX(12,12,24,24)] = 1;
	GeM1x3[_2D_IDX(13,13,24,24)] = Zc[_2D_IDX(1,7,6,9)] + Zb[_2D_IDX(1,3,6,9)] - Zb[_2D_IDX(1,7,6,9)] - Zc[_2D_IDX(1,3,6,9)];
	GeM1x3[_2D_IDX(13,14,24,24)] = Zc[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,4,6,9)] + Zb[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)];
	GeM1x3[_2D_IDX(13,15,24,24)] = -Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] + Zb[_2D_IDX(1,6,6,9)] + Zc[_2D_IDX(1,9,6,9)];
	GeM1x3[_2D_IDX(13,16,24,24)] = 2 * Zc[_2D_IDX(1,1,6,9)] - 2 * Zb[_2D_IDX(1,1,6,9)];
	GeM1x3[_2D_IDX(13,17,24,24)] = -2 * Zb[_2D_IDX(1,2,6,9)] + 2 * Zc[_2D_IDX(1,2,6,9)];
	GeM1x3[_2D_IDX(13,18,24,24)] = -2 * Zb[_2D_IDX(1,5,6,9)] + 2 * Zc[_2D_IDX(1,5,6,9)];
	GeM1x3[_2D_IDX(13,19,24,24)] = Zc[_2D_IDX(1,3,6,9)] + Zc[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,3,6,9)];
	GeM1x3[_2D_IDX(13,20,24,24)] = -Zb[_2D_IDX(1,4,6,9)] + Zc[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] + Zc[_2D_IDX(1,8,6,9)];
	GeM1x3[_2D_IDX(13,21,24,24)] = Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,6,6,9)] + Zc[_2D_IDX(1,9,6,9)];
	GeM1x3[_2D_IDX(14,13,24,24)] = Zc[_2D_IDX(2,7,6,9)] + Zb[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)];
	GeM1x3[_2D_IDX(14,14,24,24)] = -Zb[_2D_IDX(2,8,6,9)] - Zc[_2D_IDX(2,4,6,9)] + Zb[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,8,6,9)];
	GeM1x3[_2D_IDX(14,15,24,24)] = -Zb[_2D_IDX(2,9,6,9)] + Zc[_2D_IDX(2,9,6,9)] - Zc[_2D_IDX(2,6,6,9)] + Zb[_2D_IDX(2,6,6,9)];
	GeM1x3[_2D_IDX(14,16,24,24)] = -2 * Zb[_2D_IDX(2,1,6,9)] + 2 * Zc[_2D_IDX(2,1,6,9)];
	GeM1x3[_2D_IDX(14,17,24,24)] = 2 * Zc[_2D_IDX(2,2,6,9)] - 2 * Zb[_2D_IDX(2,2,6,9)];
	GeM1x3[_2D_IDX(14,18,24,24)] = -2 * Zb[_2D_IDX(2,5,6,9)] + 2 * Zc[_2D_IDX(2,5,6,9)];
	GeM1x3[_2D_IDX(14,19,24,24)] = -Zb[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)] + Zc[_2D_IDX(2,3,6,9)] + Zc[_2D_IDX(2,7,6,9)];
	GeM1x3[_2D_IDX(14,20,24,24)] = -Zb[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,8,6,9)] - Zb[_2D_IDX(2,8,6,9)];
	GeM1x3[_2D_IDX(14,21,24,24)] = Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zb[_2D_IDX(2,6,6,9)] + Zc[_2D_IDX(2,9,6,9)];
	GeM1x3[_2D_IDX(15,13,24,24)] = -Zc[_2D_IDX(3,3,6,9)] + Zb[_2D_IDX(3,3,6,9)] + Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)];
	GeM1x3[_2D_IDX(15,14,24,24)] = Zb[_2D_IDX(3,4,6,9)] - Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)] + Zc[_2D_IDX(3,8,6,9)];
	GeM1x3[_2D_IDX(15,15,24,24)] = Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)] + Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,6,6,9)];
	GeM1x3[_2D_IDX(15,16,24,24)] = 2 * Zc[_2D_IDX(3,1,6,9)] - 2 * Zb[_2D_IDX(3,1,6,9)];
	GeM1x3[_2D_IDX(15,17,24,24)] = -2 * Zb[_2D_IDX(3,2,6,9)] + 2 * Zc[_2D_IDX(3,2,6,9)];
	GeM1x3[_2D_IDX(15,18,24,24)] = -2 * Zb[_2D_IDX(3,5,6,9)] + 2 * Zc[_2D_IDX(3,5,6,9)];
	GeM1x3[_2D_IDX(15,19,24,24)] = Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,3,6,9)] - Zb[_2D_IDX(3,7,6,9)] + Zc[_2D_IDX(3,3,6,9)];
	GeM1x3[_2D_IDX(15,20,24,24)] = -Zb[_2D_IDX(3,4,6,9)] + Zc[_2D_IDX(3,8,6,9)] + Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)];
	GeM1x3[_2D_IDX(15,21,24,24)] = -Zb[_2D_IDX(3,9,6,9)] + Zc[_2D_IDX(3,6,6,9)] - Zb[_2D_IDX(3,6,6,9)] + Zc[_2D_IDX(3,9,6,9)];
	GeM1x3[_2D_IDX(16,13,24,24)] = -Zc[_2D_IDX(4,3,6,9)] + Zb[_2D_IDX(4,3,6,9)] + Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,7,6,9)];
	GeM1x3[_2D_IDX(16,14,24,24)] = Zc[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,8,6,9)] + Zb[_2D_IDX(4,4,6,9)] - Zc[_2D_IDX(4,4,6,9)];
	GeM1x3[_2D_IDX(16,15,24,24)] = -Zb[_2D_IDX(4,9,6,9)] - Zc[_2D_IDX(4,6,6,9)] + Zc[_2D_IDX(4,9,6,9)] + Zb[_2D_IDX(4,6,6,9)];
	GeM1x3[_2D_IDX(16,16,24,24)] = 2 * Zc[_2D_IDX(4,1,6,9)] - 2 * Zb[_2D_IDX(4,1,6,9)];
	GeM1x3[_2D_IDX(16,17,24,24)] = 2 * Zc[_2D_IDX(4,2,6,9)] - 2 * Zb[_2D_IDX(4,2,6,9)];
	GeM1x3[_2D_IDX(16,18,24,24)] = 2 * Zc[_2D_IDX(4,5,6,9)] - 2 * Zb[_2D_IDX(4,5,6,9)];
	GeM1x3[_2D_IDX(16,19,24,24)] = -Zb[_2D_IDX(4,7,6,9)] + Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,3,6,9)] + Zc[_2D_IDX(4,3,6,9)];
	GeM1x3[_2D_IDX(16,20,24,24)] = -Zb[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,8,6,9)];
	GeM1x3[_2D_IDX(16,21,24,24)] = Zc[_2D_IDX(4,9,6,9)] - Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)] + Zc[_2D_IDX(4,6,6,9)];
	GeM1x3[_2D_IDX(17,13,24,24)] = Zc[_2D_IDX(5,7,6,9)] - Zb[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,3,6,9)] + Zb[_2D_IDX(5,3,6,9)];
	GeM1x3[_2D_IDX(17,14,24,24)] = -Zc[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zb[_2D_IDX(5,4,6,9)];
	GeM1x3[_2D_IDX(17,15,24,24)] = -Zb[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,6,6,9)] + Zb[_2D_IDX(5,6,6,9)];
	GeM1x3[_2D_IDX(17,16,24,24)] = -2 * Zb[_2D_IDX(5,1,6,9)] + 2 * Zc[_2D_IDX(5,1,6,9)];
	GeM1x3[_2D_IDX(17,17,24,24)] = -2 * Zb[_2D_IDX(5,2,6,9)] + 2 * Zc[_2D_IDX(5,2,6,9)];
	GeM1x3[_2D_IDX(17,18,24,24)] = -2 * Zb[_2D_IDX(5,5,6,9)] + 2 * Zc[_2D_IDX(5,5,6,9)];
	GeM1x3[_2D_IDX(17,19,24,24)] = -Zb[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,3,6,9)];
	GeM1x3[_2D_IDX(17,20,24,24)] = -Zb[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zc[_2D_IDX(5,4,6,9)];
	GeM1x3[_2D_IDX(17,21,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zb[_2D_IDX(5,6,6,9)] + Zc[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,6,6,9)];
	GeM1x3[_2D_IDX(18,13,24,24)] = Zc[_2D_IDX(6,7,6,9)] - Zb[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,3,6,9)] + Zb[_2D_IDX(6,3,6,9)];
	GeM1x3[_2D_IDX(18,14,24,24)] = -Zc[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zb[_2D_IDX(6,4,6,9)];
	GeM1x3[_2D_IDX(18,15,24,24)] = -Zb[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,6,6,9)] + Zb[_2D_IDX(6,6,6,9)];
	GeM1x3[_2D_IDX(18,16,24,24)] = -2 * Zb[_2D_IDX(6,1,6,9)] + 2 * Zc[_2D_IDX(6,1,6,9)];
	GeM1x3[_2D_IDX(18,17,24,24)] = -2 * Zb[_2D_IDX(6,2,6,9)] + 2 * Zc[_2D_IDX(6,2,6,9)];
	GeM1x3[_2D_IDX(18,18,24,24)] = -2 * Zb[_2D_IDX(6,5,6,9)] + 2 * Zc[_2D_IDX(6,5,6,9)];
	GeM1x3[_2D_IDX(18,19,24,24)] = -Zb[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,3,6,9)];
	GeM1x3[_2D_IDX(18,20,24,24)] = -Zb[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zc[_2D_IDX(6,4,6,9)];
	GeM1x3[_2D_IDX(18,21,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zb[_2D_IDX(6,6,6,9)] + Zc[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,6,6,9)];
	GeM1x3[_2D_IDX(19,16,24,24)] = Zc[_2D_IDX(1,7,6,9)] + Zb[_2D_IDX(1,3,6,9)] - Zb[_2D_IDX(1,7,6,9)] - Zc[_2D_IDX(1,3,6,9)];
	GeM1x3[_2D_IDX(19,17,24,24)] = Zc[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,4,6,9)] + Zb[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)];
	GeM1x3[_2D_IDX(19,18,24,24)] = -Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] + Zb[_2D_IDX(1,6,6,9)] + Zc[_2D_IDX(1,9,6,9)];
	GeM1x3[_2D_IDX(19,19,24,24)] = 2 * Zc[_2D_IDX(1,1,6,9)] - 2 * Zb[_2D_IDX(1,1,6,9)];
	GeM1x3[_2D_IDX(19,20,24,24)] = -2 * Zb[_2D_IDX(1,2,6,9)] + 2 * Zc[_2D_IDX(1,2,6,9)];
	GeM1x3[_2D_IDX(19,21,24,24)] = -2 * Zb[_2D_IDX(1,5,6,9)] + 2 * Zc[_2D_IDX(1,5,6,9)];
	GeM1x3[_2D_IDX(19,22,24,24)] = Zc[_2D_IDX(1,3,6,9)] + Zc[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,3,6,9)];
	GeM1x3[_2D_IDX(19,23,24,24)] = -Zb[_2D_IDX(1,4,6,9)] + Zc[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] + Zc[_2D_IDX(1,8,6,9)];
	GeM1x3[_2D_IDX(19,24,24,24)] = Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,6,6,9)] + Zc[_2D_IDX(1,9,6,9)];
	GeM1x3[_2D_IDX(20,16,24,24)] = Zc[_2D_IDX(2,7,6,9)] + Zb[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)];
	GeM1x3[_2D_IDX(20,17,24,24)] = -Zb[_2D_IDX(2,8,6,9)] - Zc[_2D_IDX(2,4,6,9)] + Zb[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,8,6,9)];
	GeM1x3[_2D_IDX(20,18,24,24)] = -Zb[_2D_IDX(2,9,6,9)] + Zc[_2D_IDX(2,9,6,9)] - Zc[_2D_IDX(2,6,6,9)] + Zb[_2D_IDX(2,6,6,9)];
	GeM1x3[_2D_IDX(20,19,24,24)] = -2 * Zb[_2D_IDX(2,1,6,9)] + 2 * Zc[_2D_IDX(2,1,6,9)];
	GeM1x3[_2D_IDX(20,20,24,24)] = 2 * Zc[_2D_IDX(2,2,6,9)] - 2 * Zb[_2D_IDX(2,2,6,9)];
	GeM1x3[_2D_IDX(20,21,24,24)] = -2 * Zb[_2D_IDX(2,5,6,9)] + 2 * Zc[_2D_IDX(2,5,6,9)];
	GeM1x3[_2D_IDX(20,22,24,24)] = -Zb[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)] + Zc[_2D_IDX(2,3,6,9)] + Zc[_2D_IDX(2,7,6,9)];
	GeM1x3[_2D_IDX(20,23,24,24)] = -Zb[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,4,6,9)] + Zc[_2D_IDX(2,8,6,9)] - Zb[_2D_IDX(2,8,6,9)];
	GeM1x3[_2D_IDX(20,24,24,24)] = Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zb[_2D_IDX(2,6,6,9)] + Zc[_2D_IDX(2,9,6,9)];
	GeM1x3[_2D_IDX(21,16,24,24)] = -Zc[_2D_IDX(3,3,6,9)] + Zb[_2D_IDX(3,3,6,9)] + Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)];
	GeM1x3[_2D_IDX(21,17,24,24)] = Zb[_2D_IDX(3,4,6,9)] - Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)] + Zc[_2D_IDX(3,8,6,9)];
	GeM1x3[_2D_IDX(21,18,24,24)] = Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)] + Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,6,6,9)];
	GeM1x3[_2D_IDX(21,19,24,24)] = 2 * Zc[_2D_IDX(3,1,6,9)] - 2 * Zb[_2D_IDX(3,1,6,9)];
	GeM1x3[_2D_IDX(21,20,24,24)] = -2 * Zb[_2D_IDX(3,2,6,9)] + 2 * Zc[_2D_IDX(3,2,6,9)];
	GeM1x3[_2D_IDX(21,21,24,24)] = -2 * Zb[_2D_IDX(3,5,6,9)] + 2 * Zc[_2D_IDX(3,5,6,9)];
	GeM1x3[_2D_IDX(21,22,24,24)] = Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,3,6,9)] - Zb[_2D_IDX(3,7,6,9)] + Zc[_2D_IDX(3,3,6,9)];
	GeM1x3[_2D_IDX(21,23,24,24)] = -Zb[_2D_IDX(3,4,6,9)] + Zc[_2D_IDX(3,8,6,9)] + Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)];
	GeM1x3[_2D_IDX(21,24,24,24)] = -Zb[_2D_IDX(3,9,6,9)] + Zc[_2D_IDX(3,6,6,9)] - Zb[_2D_IDX(3,6,6,9)] + Zc[_2D_IDX(3,9,6,9)];
	GeM1x3[_2D_IDX(22,16,24,24)] = -Zc[_2D_IDX(4,3,6,9)] + Zb[_2D_IDX(4,3,6,9)] + Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,7,6,9)];
	GeM1x3[_2D_IDX(22,17,24,24)] = Zc[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,8,6,9)] + Zb[_2D_IDX(4,4,6,9)] - Zc[_2D_IDX(4,4,6,9)];
	GeM1x3[_2D_IDX(22,18,24,24)] = -Zb[_2D_IDX(4,9,6,9)] - Zc[_2D_IDX(4,6,6,9)] + Zc[_2D_IDX(4,9,6,9)] + Zb[_2D_IDX(4,6,6,9)];
	GeM1x3[_2D_IDX(22,19,24,24)] = 2 * Zc[_2D_IDX(4,1,6,9)] - 2 * Zb[_2D_IDX(4,1,6,9)];
	GeM1x3[_2D_IDX(22,20,24,24)] = 2 * Zc[_2D_IDX(4,2,6,9)] - 2 * Zb[_2D_IDX(4,2,6,9)];
	GeM1x3[_2D_IDX(22,21,24,24)] = 2 * Zc[_2D_IDX(4,5,6,9)] - 2 * Zb[_2D_IDX(4,5,6,9)];
	GeM1x3[_2D_IDX(22,22,24,24)] = -Zb[_2D_IDX(4,7,6,9)] + Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,3,6,9)] + Zc[_2D_IDX(4,3,6,9)];
	GeM1x3[_2D_IDX(22,23,24,24)] = -Zb[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,8,6,9)];
	GeM1x3[_2D_IDX(22,24,24,24)] = Zc[_2D_IDX(4,9,6,9)] - Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)] + Zc[_2D_IDX(4,6,6,9)];
	GeM1x3[_2D_IDX(23,16,24,24)] = Zc[_2D_IDX(5,7,6,9)] - Zb[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,3,6,9)] + Zb[_2D_IDX(5,3,6,9)];
	GeM1x3[_2D_IDX(23,17,24,24)] = -Zc[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zb[_2D_IDX(5,4,6,9)];
	GeM1x3[_2D_IDX(23,18,24,24)] = -Zb[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,6,6,9)] + Zb[_2D_IDX(5,6,6,9)];
	GeM1x3[_2D_IDX(23,19,24,24)] = -2 * Zb[_2D_IDX(5,1,6,9)] + 2 * Zc[_2D_IDX(5,1,6,9)];
	GeM1x3[_2D_IDX(23,20,24,24)] = -2 * Zb[_2D_IDX(5,2,6,9)] + 2 * Zc[_2D_IDX(5,2,6,9)];
	GeM1x3[_2D_IDX(23,21,24,24)] = -2 * Zb[_2D_IDX(5,5,6,9)] + 2 * Zc[_2D_IDX(5,5,6,9)];
	GeM1x3[_2D_IDX(23,22,24,24)] = -Zb[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,3,6,9)];
	GeM1x3[_2D_IDX(23,23,24,24)] = -Zb[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zc[_2D_IDX(5,4,6,9)];
	GeM1x3[_2D_IDX(23,24,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zb[_2D_IDX(5,6,6,9)] + Zc[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,6,6,9)];
	GeM1x3[_2D_IDX(24,16,24,24)] = Zc[_2D_IDX(6,7,6,9)] - Zb[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,3,6,9)] + Zb[_2D_IDX(6,3,6,9)];
	GeM1x3[_2D_IDX(24,17,24,24)] = -Zc[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zb[_2D_IDX(6,4,6,9)];
	GeM1x3[_2D_IDX(24,18,24,24)] = -Zb[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,6,6,9)] + Zb[_2D_IDX(6,6,6,9)];
	GeM1x3[_2D_IDX(24,19,24,24)] = -2 * Zb[_2D_IDX(6,1,6,9)] + 2 * Zc[_2D_IDX(6,1,6,9)];
	GeM1x3[_2D_IDX(24,20,24,24)] = -2 * Zb[_2D_IDX(6,2,6,9)] + 2 * Zc[_2D_IDX(6,2,6,9)];
	GeM1x3[_2D_IDX(24,21,24,24)] = -2 * Zb[_2D_IDX(6,5,6,9)] + 2 * Zc[_2D_IDX(6,5,6,9)];
	GeM1x3[_2D_IDX(24,22,24,24)] = -Zb[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,3,6,9)];
	GeM1x3[_2D_IDX(24,23,24,24)] = -Zb[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zc[_2D_IDX(6,4,6,9)];
	GeM1x3[_2D_IDX(24,24,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zb[_2D_IDX(6,6,6,9)] + Zc[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,6,6,9)];

	GeM2x3[_2D_IDX(1,13,24,24)] = 1;
	GeM2x3[_2D_IDX(2,14,24,24)] = 1;
	GeM2x3[_2D_IDX(3,15,24,24)] = 1;
	GeM2x3[_2D_IDX(4,16,24,24)] = 1;
	GeM2x3[_2D_IDX(5,17,24,24)] = 1;
	GeM2x3[_2D_IDX(6,18,24,24)] = 1;
	GeM2x3[_2D_IDX(7,19,24,24)] = 1;
	GeM2x3[_2D_IDX(8,20,24,24)] = 1;
	GeM2x3[_2D_IDX(9,21,24,24)] = 1;
	GeM2x3[_2D_IDX(10,22,24,24)] = 1;
	GeM2x3[_2D_IDX(11,23,24,24)] = 1;
	GeM2x3[_2D_IDX(12,24,24,24)] = 1;
	GeM2x3[_2D_IDX(13,1,24,24)] = -Zc[_2D_IDX(1,7,6,9)] + Zb[_2D_IDX(1,3,6,9)] - Zb[_2D_IDX(1,7,6,9)] + Zc[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(13,2,24,24)] = Zc[_2D_IDX(1,4,6,9)] + Zb[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(13,3,24,24)] = Zc[_2D_IDX(1,6,6,9)] + Zb[_2D_IDX(1,6,6,9)] - Zc[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(13,4,24,24)] = -2 * Zc[_2D_IDX(1,1,6,9)] - 2 * Zb[_2D_IDX(1,1,6,9)];
	GeM2x3[_2D_IDX(13,5,24,24)] = -2 * Zb[_2D_IDX(1,2,6,9)] - 2 * Zc[_2D_IDX(1,2,6,9)];
	GeM2x3[_2D_IDX(13,6,24,24)] = -2 * Zb[_2D_IDX(1,5,6,9)] - 2 * Zc[_2D_IDX(1,5,6,9)];
	GeM2x3[_2D_IDX(13,7,24,24)] = -Zb[_2D_IDX(1,7,6,9)] - Zc[_2D_IDX(1,3,6,9)] - Zc[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(13,8,24,24)] = -Zc[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] - Zb[_2D_IDX(1,4,6,9)] - Zc[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(13,9,24,24)] = -Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,6,6,9)] - Zc[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(13,13,24,24)] = 2 * Za[_2D_IDX(1,3,6,9)] - 2 * Za[_2D_IDX(1,7,6,9)];
	GeM2x3[_2D_IDX(13,14,24,24)] = 2 * Za[_2D_IDX(1,4,6,9)] - 2 * Za[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(13,15,24,24)] = 2 * Za[_2D_IDX(1,6,6,9)] - 2 * Za[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(13,16,24,24)] = -4 * Za[_2D_IDX(1,1,6,9)];
	GeM2x3[_2D_IDX(13,17,24,24)] = -4 * Za[_2D_IDX(1,2,6,9)];
	GeM2x3[_2D_IDX(13,18,24,24)] = -4 * Za[_2D_IDX(1,5,6,9)];
	GeM2x3[_2D_IDX(13,19,24,24)] = -2 * Za[_2D_IDX(1,7,6,9)] - 2 * Za[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(13,20,24,24)] = -2 * Za[_2D_IDX(1,4,6,9)] - 2 * Za[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(13,21,24,24)] = -2 * Za[_2D_IDX(1,6,6,9)] - 2 * Za[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(14,1,24,24)] = -Zc[_2D_IDX(2,7,6,9)] + Zb[_2D_IDX(2,3,6,9)] + Zc[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(14,2,24,24)] = Zb[_2D_IDX(2,4,6,9)] - Zb[_2D_IDX(2,8,6,9)] + Zc[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(14,3,24,24)] = Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zc[_2D_IDX(2,9,6,9)] + Zb[_2D_IDX(2,6,6,9)];
	GeM2x3[_2D_IDX(14,4,24,24)] = -2 * Zb[_2D_IDX(2,1,6,9)] - 2 * Zc[_2D_IDX(2,1,6,9)];
	GeM2x3[_2D_IDX(14,5,24,24)] = -2 * Zc[_2D_IDX(2,2,6,9)] - 2 * Zb[_2D_IDX(2,2,6,9)];
	GeM2x3[_2D_IDX(14,6,24,24)] = -2 * Zb[_2D_IDX(2,5,6,9)] - 2 * Zc[_2D_IDX(2,5,6,9)];
	GeM2x3[_2D_IDX(14,7,24,24)] = -Zb[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)] - Zc[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(14,8,24,24)] = -Zb[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,8,6,9)] - Zb[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(14,9,24,24)] = -Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zb[_2D_IDX(2,6,6,9)] - Zc[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(14,13,24,24)] = -2 * Za[_2D_IDX(2,7,6,9)] + 2 * Za[_2D_IDX(2,3,6,9)];
	GeM2x3[_2D_IDX(14,14,24,24)] = 2 * Za[_2D_IDX(2,4,6,9)] - 2 * Za[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(14,15,24,24)] = 2 * Za[_2D_IDX(2,6,6,9)] - 2 * Za[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(14,16,24,24)] = -4 * Za[_2D_IDX(2,1,6,9)];
	GeM2x3[_2D_IDX(14,17,24,24)] = -4 * Za[_2D_IDX(2,2,6,9)];
	GeM2x3[_2D_IDX(14,18,24,24)] = -4 * Za[_2D_IDX(2,5,6,9)];
	GeM2x3[_2D_IDX(14,19,24,24)] = -2 * Za[_2D_IDX(2,3,6,9)] - 2 * Za[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(14,20,24,24)] = -2 * Za[_2D_IDX(2,8,6,9)] - 2 * Za[_2D_IDX(2,4,6,9)];
	GeM2x3[_2D_IDX(14,21,24,24)] = -2 * Za[_2D_IDX(2,6,6,9)] - 2 * Za[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(15,1,24,24)] = Zc[_2D_IDX(3,3,6,9)] + Zb[_2D_IDX(3,3,6,9)] - Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)];
	GeM2x3[_2D_IDX(15,2,24,24)] = -Zb[_2D_IDX(3,8,6,9)] + Zb[_2D_IDX(3,4,6,9)] + Zc[_2D_IDX(3,4,6,9)] - Zc[_2D_IDX(3,8,6,9)];
	GeM2x3[_2D_IDX(15,3,24,24)] = Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)] + Zc[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(15,4,24,24)] = -2 * Zc[_2D_IDX(3,1,6,9)] - 2 * Zb[_2D_IDX(3,1,6,9)];
	GeM2x3[_2D_IDX(15,5,24,24)] = -2 * Zb[_2D_IDX(3,2,6,9)] - 2 * Zc[_2D_IDX(3,2,6,9)];
	GeM2x3[_2D_IDX(15,6,24,24)] = -2 * Zb[_2D_IDX(3,5,6,9)] - 2 * Zc[_2D_IDX(3,5,6,9)];
	GeM2x3[_2D_IDX(15,7,24,24)] = -Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)] - Zc[_2D_IDX(3,3,6,9)] - Zb[_2D_IDX(3,3,6,9)];
	GeM2x3[_2D_IDX(15,8,24,24)] = -Zc[_2D_IDX(3,8,6,9)] - Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)] - Zb[_2D_IDX(3,4,6,9)];
	GeM2x3[_2D_IDX(15,9,24,24)] = -Zc[_2D_IDX(3,6,6,9)] - Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)];
	GeM2x3[_2D_IDX(15,13,24,24)] = 2 * Za[_2D_IDX(3,3,6,9)] - 2 * Za[_2D_IDX(3,7,6,9)];
	GeM2x3[_2D_IDX(15,14,24,24)] = 2 * Za[_2D_IDX(3,4,6,9)] - 2 * Za[_2D_IDX(3,8,6,9)];
	GeM2x3[_2D_IDX(15,15,24,24)] = -2 * Za[_2D_IDX(3,9,6,9)] + 2 * Za[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(15,16,24,24)] = -4 * Za[_2D_IDX(3,1,6,9)];
	GeM2x3[_2D_IDX(15,17,24,24)] = -4 * Za[_2D_IDX(3,2,6,9)];
	GeM2x3[_2D_IDX(15,18,24,24)] = -4 * Za[_2D_IDX(3,5,6,9)];
	GeM2x3[_2D_IDX(15,19,24,24)] = -2 * Za[_2D_IDX(3,7,6,9)] - 2 * Za[_2D_IDX(3,3,6,9)];
	GeM2x3[_2D_IDX(15,20,24,24)] = -2 * Za[_2D_IDX(3,8,6,9)] - 2 * Za[_2D_IDX(3,4,6,9)];
	GeM2x3[_2D_IDX(15,21,24,24)] = -2 * Za[_2D_IDX(3,9,6,9)] - 2 * Za[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(16,1,24,24)] = Zc[_2D_IDX(4,3,6,9)] - Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,7,6,9)] + Zb[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(16,2,24,24)] = -Zc[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,8,6,9)] + Zb[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(16,3,24,24)] = Zc[_2D_IDX(4,6,6,9)] - Zc[_2D_IDX(4,9,6,9)] + Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)];
	GeM2x3[_2D_IDX(16,4,24,24)] = -2 * Zc[_2D_IDX(4,1,6,9)] - 2 * Zb[_2D_IDX(4,1,6,9)];
	GeM2x3[_2D_IDX(16,5,24,24)] = -2 * Zc[_2D_IDX(4,2,6,9)] - 2 * Zb[_2D_IDX(4,2,6,9)];
	GeM2x3[_2D_IDX(16,6,24,24)] = -2 * Zc[_2D_IDX(4,5,6,9)] - 2 * Zb[_2D_IDX(4,5,6,9)];
	GeM2x3[_2D_IDX(16,7,24,24)] = -Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,3,6,9)] - Zc[_2D_IDX(4,3,6,9)] - Zb[_2D_IDX(4,7,6,9)];
	GeM2x3[_2D_IDX(16,8,24,24)] = -Zb[_2D_IDX(4,4,6,9)] - Zb[_2D_IDX(4,8,6,9)] - Zc[_2D_IDX(4,8,6,9)] - Zc[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(16,9,24,24)] = -Zc[_2D_IDX(4,9,6,9)] - Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)] - Zc[_2D_IDX(4,6,6,9)];
	GeM2x3[_2D_IDX(16,13,24,24)] = -2 * Za[_2D_IDX(4,7,6,9)] + 2 * Za[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(16,14,24,24)] = 2 * Za[_2D_IDX(4,4,6,9)] - 2 * Za[_2D_IDX(4,8,6,9)];
	GeM2x3[_2D_IDX(16,15,24,24)] = 2 * Za[_2D_IDX(4,6,6,9)] - 2 * Za[_2D_IDX(4,9,6,9)];
	GeM2x3[_2D_IDX(16,16,24,24)] = -4 * Za[_2D_IDX(4,1,6,9)];
	GeM2x3[_2D_IDX(16,17,24,24)] = -4 * Za[_2D_IDX(4,2,6,9)];
	GeM2x3[_2D_IDX(16,18,24,24)] = -4 * Za[_2D_IDX(4,5,6,9)];
	GeM2x3[_2D_IDX(16,19,24,24)] = -2 * Za[_2D_IDX(4,7,6,9)] - 2 * Za[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(16,20,24,24)] = -2 * Za[_2D_IDX(4,8,6,9)] - 2 * Za[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(16,21,24,24)] = -2 * Za[_2D_IDX(4,9,6,9)] - 2 * Za[_2D_IDX(4,6,6,9)];
	GeM2x3[_2D_IDX(17,1,24,24)] = -Zc[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,3,6,9)] + Zb[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,7,6,9)];
	GeM2x3[_2D_IDX(17,2,24,24)] = -Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zb[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(17,3,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,6,6,9)] + Zb[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(17,4,24,24)] = -2 * Zb[_2D_IDX(5,1,6,9)] - 2 * Zc[_2D_IDX(5,1,6,9)];
	GeM2x3[_2D_IDX(17,5,24,24)] = -2 * Zc[_2D_IDX(5,2,6,9)] - 2 * Zb[_2D_IDX(5,2,6,9)];
	GeM2x3[_2D_IDX(17,6,24,24)] = -2 * Zb[_2D_IDX(5,5,6,9)] - 2 * Zc[_2D_IDX(5,5,6,9)];
	GeM2x3[_2D_IDX(17,7,24,24)] = -Zb[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,3,6,9)];
	GeM2x3[_2D_IDX(17,8,24,24)] = -Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,4,6,9)] - Zc[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(17,9,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,6,6,9)] - Zb[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(17,13,24,24)] = 2 * Za[_2D_IDX(5,3,6,9)] - 2 * Za[_2D_IDX(5,7,6,9)];
	GeM2x3[_2D_IDX(17,14,24,24)] = -2 * Za[_2D_IDX(5,8,6,9)] + 2 * Za[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(17,15,24,24)] = 2 * Za[_2D_IDX(5,6,6,9)] - 2 * Za[_2D_IDX(5,9,6,9)];
	GeM2x3[_2D_IDX(17,16,24,24)] = -4 * Za[_2D_IDX(5,1,6,9)];
	GeM2x3[_2D_IDX(17,17,24,24)] = -4 * Za[_2D_IDX(5,2,6,9)];
	GeM2x3[_2D_IDX(17,18,24,24)] = -4 * Za[_2D_IDX(5,5,6,9)];
	GeM2x3[_2D_IDX(17,19,24,24)] = -2 * Za[_2D_IDX(5,7,6,9)] - 2 * Za[_2D_IDX(5,3,6,9)];
	GeM2x3[_2D_IDX(17,20,24,24)] = -2 * Za[_2D_IDX(5,8,6,9)] - 2 * Za[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(17,21,24,24)] = -2 * Za[_2D_IDX(5,9,6,9)] - 2 * Za[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(18,1,24,24)] = -Zc[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,3,6,9)] + Zb[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,7,6,9)];
	GeM2x3[_2D_IDX(18,2,24,24)] = -Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zb[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(18,3,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,6,6,9)] + Zb[_2D_IDX(6,6,6,9)];
	GeM2x3[_2D_IDX(18,4,24,24)] = -2 * Zb[_2D_IDX(6,1,6,9)] - 2 * Zc[_2D_IDX(6,1,6,9)];
	GeM2x3[_2D_IDX(18,5,24,24)] = -2 * Zc[_2D_IDX(6,2,6,9)] - 2 * Zb[_2D_IDX(6,2,6,9)];
	GeM2x3[_2D_IDX(18,6,24,24)] = -2 * Zb[_2D_IDX(6,5,6,9)] - 2 * Zc[_2D_IDX(6,5,6,9)];
	GeM2x3[_2D_IDX(18,7,24,24)] = -Zb[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,3,6,9)];
	GeM2x3[_2D_IDX(18,8,24,24)] = -Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,4,6,9)] - Zc[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(18,9,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,6,6,9)] - Zb[_2D_IDX(6,6,6,9)];
	GeM2x3[_2D_IDX(18,13,24,24)] = 2 * Za[_2D_IDX(6,3,6,9)] - 2 * Za[_2D_IDX(6,7,6,9)];
	GeM2x3[_2D_IDX(18,14,24,24)] = -2 * Za[_2D_IDX(6,8,6,9)] + 2 * Za[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(18,15,24,24)] = 2 * Za[_2D_IDX(6,6,6,9)] - 2 * Za[_2D_IDX(6,9,6,9)];
	GeM2x3[_2D_IDX(18,16,24,24)] = -4 * Za[_2D_IDX(6,1,6,9)];
	GeM2x3[_2D_IDX(18,17,24,24)] = -4 * Za[_2D_IDX(6,2,6,9)];
	GeM2x3[_2D_IDX(18,18,24,24)] = -4 * Za[_2D_IDX(6,5,6,9)];
	GeM2x3[_2D_IDX(18,19,24,24)] = -2 * Za[_2D_IDX(6,7,6,9)] - 2 * Za[_2D_IDX(6,3,6,9)];
	GeM2x3[_2D_IDX(18,20,24,24)] = -2 * Za[_2D_IDX(6,8,6,9)] - 2 * Za[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(18,21,24,24)] = -2 * Za[_2D_IDX(6,9,6,9)] - 2 * Za[_2D_IDX(6,6,6,9)];
	GeM2x3[_2D_IDX(19,4,24,24)] = -Zc[_2D_IDX(1,7,6,9)] + Zb[_2D_IDX(1,3,6,9)] - Zb[_2D_IDX(1,7,6,9)] + Zc[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(19,5,24,24)] = Zc[_2D_IDX(1,4,6,9)] + Zb[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(19,6,24,24)] = Zc[_2D_IDX(1,6,6,9)] + Zb[_2D_IDX(1,6,6,9)] - Zc[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(19,7,24,24)] = -2 * Zc[_2D_IDX(1,1,6,9)] - 2 * Zb[_2D_IDX(1,1,6,9)];
	GeM2x3[_2D_IDX(19,8,24,24)] = -2 * Zb[_2D_IDX(1,2,6,9)] - 2 * Zc[_2D_IDX(1,2,6,9)];
	GeM2x3[_2D_IDX(19,9,24,24)] = -2 * Zb[_2D_IDX(1,5,6,9)] - 2 * Zc[_2D_IDX(1,5,6,9)];
	GeM2x3[_2D_IDX(19,10,24,24)] = -Zb[_2D_IDX(1,7,6,9)] - Zc[_2D_IDX(1,3,6,9)] - Zc[_2D_IDX(1,7,6,9)] - Zb[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(19,11,24,24)] = -Zc[_2D_IDX(1,4,6,9)] - Zb[_2D_IDX(1,8,6,9)] - Zb[_2D_IDX(1,4,6,9)] - Zc[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(19,12,24,24)] = -Zc[_2D_IDX(1,6,6,9)] - Zb[_2D_IDX(1,9,6,9)] - Zb[_2D_IDX(1,6,6,9)] - Zc[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(19,16,24,24)] = 2 * Za[_2D_IDX(1,3,6,9)] - 2 * Za[_2D_IDX(1,7,6,9)];
	GeM2x3[_2D_IDX(19,17,24,24)] = 2 * Za[_2D_IDX(1,4,6,9)] - 2 * Za[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(19,18,24,24)] = 2 * Za[_2D_IDX(1,6,6,9)] - 2 * Za[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(19,19,24,24)] = -4 * Za[_2D_IDX(1,1,6,9)];
	GeM2x3[_2D_IDX(19,20,24,24)] = -4 * Za[_2D_IDX(1,2,6,9)];
	GeM2x3[_2D_IDX(19,21,24,24)] = -4 * Za[_2D_IDX(1,5,6,9)];
	GeM2x3[_2D_IDX(19,22,24,24)] = -2 * Za[_2D_IDX(1,7,6,9)] - 2 * Za[_2D_IDX(1,3,6,9)];
	GeM2x3[_2D_IDX(19,23,24,24)] = -2 * Za[_2D_IDX(1,4,6,9)] - 2 * Za[_2D_IDX(1,8,6,9)];
	GeM2x3[_2D_IDX(19,24,24,24)] = -2 * Za[_2D_IDX(1,6,6,9)] - 2 * Za[_2D_IDX(1,9,6,9)];
	GeM2x3[_2D_IDX(20,4,24,24)] = -Zc[_2D_IDX(2,7,6,9)] + Zb[_2D_IDX(2,3,6,9)] + Zc[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(20,5,24,24)] = Zb[_2D_IDX(2,4,6,9)] - Zb[_2D_IDX(2,8,6,9)] + Zc[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(20,6,24,24)] = Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zc[_2D_IDX(2,9,6,9)] + Zb[_2D_IDX(2,6,6,9)];
	GeM2x3[_2D_IDX(20,7,24,24)] = -2 * Zb[_2D_IDX(2,1,6,9)] - 2 * Zc[_2D_IDX(2,1,6,9)];
	GeM2x3[_2D_IDX(20,8,24,24)] = -2 * Zc[_2D_IDX(2,2,6,9)] - 2 * Zb[_2D_IDX(2,2,6,9)];
	GeM2x3[_2D_IDX(20,9,24,24)] = -2 * Zb[_2D_IDX(2,5,6,9)] - 2 * Zc[_2D_IDX(2,5,6,9)];
	GeM2x3[_2D_IDX(20,10,24,24)] = -Zb[_2D_IDX(2,3,6,9)] - Zb[_2D_IDX(2,7,6,9)] - Zc[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(20,11,24,24)] = -Zb[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,8,6,9)] - Zb[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(20,12,24,24)] = -Zc[_2D_IDX(2,6,6,9)] - Zb[_2D_IDX(2,9,6,9)] - Zb[_2D_IDX(2,6,6,9)] - Zc[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(20,16,24,24)] = -2 * Za[_2D_IDX(2,7,6,9)] + 2 * Za[_2D_IDX(2,3,6,9)];
	GeM2x3[_2D_IDX(20,17,24,24)] = 2 * Za[_2D_IDX(2,4,6,9)] - 2 * Za[_2D_IDX(2,8,6,9)];
	GeM2x3[_2D_IDX(20,18,24,24)] = 2 * Za[_2D_IDX(2,6,6,9)] - 2 * Za[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(20,19,24,24)] = -4 * Za[_2D_IDX(2,1,6,9)];
	GeM2x3[_2D_IDX(20,20,24,24)] = -4 * Za[_2D_IDX(2,2,6,9)];
	GeM2x3[_2D_IDX(20,21,24,24)] = -4 * Za[_2D_IDX(2,5,6,9)];
	GeM2x3[_2D_IDX(20,22,24,24)] = -2 * Za[_2D_IDX(2,3,6,9)] - 2 * Za[_2D_IDX(2,7,6,9)];
	GeM2x3[_2D_IDX(20,23,24,24)] = -2 * Za[_2D_IDX(2,8,6,9)] - 2 * Za[_2D_IDX(2,4,6,9)];
	GeM2x3[_2D_IDX(20,24,24,24)] = -2 * Za[_2D_IDX(2,6,6,9)] - 2 * Za[_2D_IDX(2,9,6,9)];
	GeM2x3[_2D_IDX(21,4,24,24)] = Zc[_2D_IDX(3,3,6,9)] + Zb[_2D_IDX(3,3,6,9)] - Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)];
	GeM2x3[_2D_IDX(21,5,24,24)] = -Zb[_2D_IDX(3,8,6,9)] + Zb[_2D_IDX(3,4,6,9)] + Zc[_2D_IDX(3,4,6,9)] - Zc[_2D_IDX(3,8,6,9)];
	GeM2x3[_2D_IDX(21,6,24,24)] = Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)] + Zc[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(21,7,24,24)] = -2 * Zc[_2D_IDX(3,1,6,9)] - 2 * Zb[_2D_IDX(3,1,6,9)];
	GeM2x3[_2D_IDX(21,8,24,24)] = -2 * Zb[_2D_IDX(3,2,6,9)] - 2 * Zc[_2D_IDX(3,2,6,9)];
	GeM2x3[_2D_IDX(21,9,24,24)] = -2 * Zb[_2D_IDX(3,5,6,9)] - 2 * Zc[_2D_IDX(3,5,6,9)];
	GeM2x3[_2D_IDX(21,10,24,24)] = -Zc[_2D_IDX(3,7,6,9)] - Zb[_2D_IDX(3,7,6,9)] - Zc[_2D_IDX(3,3,6,9)] - Zb[_2D_IDX(3,3,6,9)];
	GeM2x3[_2D_IDX(21,11,24,24)] = -Zc[_2D_IDX(3,8,6,9)] - Zc[_2D_IDX(3,4,6,9)] - Zb[_2D_IDX(3,8,6,9)] - Zb[_2D_IDX(3,4,6,9)];
	GeM2x3[_2D_IDX(21,12,24,24)] = -Zc[_2D_IDX(3,6,6,9)] - Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,9,6,9)] - Zb[_2D_IDX(3,9,6,9)];
	GeM2x3[_2D_IDX(21,16,24,24)] = 2 * Za[_2D_IDX(3,3,6,9)] - 2 * Za[_2D_IDX(3,7,6,9)];
	GeM2x3[_2D_IDX(21,17,24,24)] = 2 * Za[_2D_IDX(3,4,6,9)] - 2 * Za[_2D_IDX(3,8,6,9)];
	GeM2x3[_2D_IDX(21,18,24,24)] = -2 * Za[_2D_IDX(3,9,6,9)] + 2 * Za[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(21,19,24,24)] = -4 * Za[_2D_IDX(3,1,6,9)];
	GeM2x3[_2D_IDX(21,20,24,24)] = -4 * Za[_2D_IDX(3,2,6,9)];
	GeM2x3[_2D_IDX(21,21,24,24)] = -4 * Za[_2D_IDX(3,5,6,9)];
	GeM2x3[_2D_IDX(21,22,24,24)] = -2 * Za[_2D_IDX(3,7,6,9)] - 2 * Za[_2D_IDX(3,3,6,9)];
	GeM2x3[_2D_IDX(21,23,24,24)] = -2 * Za[_2D_IDX(3,8,6,9)] - 2 * Za[_2D_IDX(3,4,6,9)];
	GeM2x3[_2D_IDX(21,24,24,24)] = -2 * Za[_2D_IDX(3,9,6,9)] - 2 * Za[_2D_IDX(3,6,6,9)];
	GeM2x3[_2D_IDX(22,4,24,24)] = Zc[_2D_IDX(4,3,6,9)] - Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,7,6,9)] + Zb[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(22,5,24,24)] = -Zc[_2D_IDX(4,8,6,9)] - Zb[_2D_IDX(4,8,6,9)] + Zb[_2D_IDX(4,4,6,9)] + Zc[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(22,6,24,24)] = Zc[_2D_IDX(4,6,6,9)] - Zc[_2D_IDX(4,9,6,9)] + Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)];
	GeM2x3[_2D_IDX(22,7,24,24)] = -2 * Zc[_2D_IDX(4,1,6,9)] - 2 * Zb[_2D_IDX(4,1,6,9)];
	GeM2x3[_2D_IDX(22,8,24,24)] = -2 * Zc[_2D_IDX(4,2,6,9)] - 2 * Zb[_2D_IDX(4,2,6,9)];
	GeM2x3[_2D_IDX(22,9,24,24)] = -2 * Zc[_2D_IDX(4,5,6,9)] - 2 * Zb[_2D_IDX(4,5,6,9)];
	GeM2x3[_2D_IDX(22,10,24,24)] = -Zc[_2D_IDX(4,7,6,9)] - Zb[_2D_IDX(4,3,6,9)] - Zc[_2D_IDX(4,3,6,9)] - Zb[_2D_IDX(4,7,6,9)];
	GeM2x3[_2D_IDX(22,11,24,24)] = -Zb[_2D_IDX(4,4,6,9)] - Zb[_2D_IDX(4,8,6,9)] - Zc[_2D_IDX(4,8,6,9)] - Zc[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(22,12,24,24)] = -Zc[_2D_IDX(4,9,6,9)] - Zb[_2D_IDX(4,6,6,9)] - Zb[_2D_IDX(4,9,6,9)] - Zc[_2D_IDX(4,6,6,9)];
	GeM2x3[_2D_IDX(22,16,24,24)] = -2 * Za[_2D_IDX(4,7,6,9)] + 2 * Za[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(22,17,24,24)] = 2 * Za[_2D_IDX(4,4,6,9)] - 2 * Za[_2D_IDX(4,8,6,9)];
	GeM2x3[_2D_IDX(22,18,24,24)] = 2 * Za[_2D_IDX(4,6,6,9)] - 2 * Za[_2D_IDX(4,9,6,9)];
	GeM2x3[_2D_IDX(22,19,24,24)] = -4 * Za[_2D_IDX(4,1,6,9)];
	GeM2x3[_2D_IDX(22,20,24,24)] = -4 * Za[_2D_IDX(4,2,6,9)];
	GeM2x3[_2D_IDX(22,21,24,24)] = -4 * Za[_2D_IDX(4,5,6,9)];
	GeM2x3[_2D_IDX(22,22,24,24)] = -2 * Za[_2D_IDX(4,7,6,9)] - 2 * Za[_2D_IDX(4,3,6,9)];
	GeM2x3[_2D_IDX(22,23,24,24)] = -2 * Za[_2D_IDX(4,8,6,9)] - 2 * Za[_2D_IDX(4,4,6,9)];
	GeM2x3[_2D_IDX(22,24,24,24)] = -2 * Za[_2D_IDX(4,9,6,9)] - 2 * Za[_2D_IDX(4,6,6,9)];
	GeM2x3[_2D_IDX(23,4,24,24)] = -Zc[_2D_IDX(5,7,6,9)] + Zc[_2D_IDX(5,3,6,9)] + Zb[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,7,6,9)];
	GeM2x3[_2D_IDX(23,5,24,24)] = -Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] + Zb[_2D_IDX(5,4,6,9)] + Zc[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(23,6,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,9,6,9)] + Zc[_2D_IDX(5,6,6,9)] + Zb[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(23,7,24,24)] = -2 * Zb[_2D_IDX(5,1,6,9)] - 2 * Zc[_2D_IDX(5,1,6,9)];
	GeM2x3[_2D_IDX(23,8,24,24)] = -2 * Zc[_2D_IDX(5,2,6,9)] - 2 * Zb[_2D_IDX(5,2,6,9)];
	GeM2x3[_2D_IDX(23,9,24,24)] = -2 * Zb[_2D_IDX(5,5,6,9)] - 2 * Zc[_2D_IDX(5,5,6,9)];
	GeM2x3[_2D_IDX(23,10,24,24)] = -Zb[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,7,6,9)] - Zc[_2D_IDX(5,3,6,9)] - Zb[_2D_IDX(5,3,6,9)];
	GeM2x3[_2D_IDX(23,11,24,24)] = -Zc[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,8,6,9)] - Zb[_2D_IDX(5,4,6,9)] - Zc[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(23,12,24,24)] = -Zb[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,9,6,9)] - Zc[_2D_IDX(5,6,6,9)] - Zb[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(23,16,24,24)] = 2 * Za[_2D_IDX(5,3,6,9)] - 2 * Za[_2D_IDX(5,7,6,9)];
	GeM2x3[_2D_IDX(23,17,24,24)] = -2 * Za[_2D_IDX(5,8,6,9)] + 2 * Za[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(23,18,24,24)] = 2 * Za[_2D_IDX(5,6,6,9)] - 2 * Za[_2D_IDX(5,9,6,9)];
	GeM2x3[_2D_IDX(23,19,24,24)] = -4 * Za[_2D_IDX(5,1,6,9)];
	GeM2x3[_2D_IDX(23,20,24,24)] = -4 * Za[_2D_IDX(5,2,6,9)];
	GeM2x3[_2D_IDX(23,21,24,24)] = -4 * Za[_2D_IDX(5,5,6,9)];
	GeM2x3[_2D_IDX(23,22,24,24)] = -2 * Za[_2D_IDX(5,7,6,9)] - 2 * Za[_2D_IDX(5,3,6,9)];
	GeM2x3[_2D_IDX(23,23,24,24)] = -2 * Za[_2D_IDX(5,8,6,9)] - 2 * Za[_2D_IDX(5,4,6,9)];
	GeM2x3[_2D_IDX(23,24,24,24)] = -2 * Za[_2D_IDX(5,9,6,9)] - 2 * Za[_2D_IDX(5,6,6,9)];
	GeM2x3[_2D_IDX(24,4,24,24)] = -Zc[_2D_IDX(6,7,6,9)] + Zc[_2D_IDX(6,3,6,9)] + Zb[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,7,6,9)];
	GeM2x3[_2D_IDX(24,5,24,24)] = -Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] + Zb[_2D_IDX(6,4,6,9)] + Zc[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(24,6,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,9,6,9)] + Zc[_2D_IDX(6,6,6,9)] + Zb[_2D_IDX(6,6,6,9)];
	GeM2x3[_2D_IDX(24,7,24,24)] = -2 * Zb[_2D_IDX(6,1,6,9)] - 2 * Zc[_2D_IDX(6,1,6,9)];
	GeM2x3[_2D_IDX(24,8,24,24)] = -2 * Zc[_2D_IDX(6,2,6,9)] - 2 * Zb[_2D_IDX(6,2,6,9)];
	GeM2x3[_2D_IDX(24,9,24,24)] = -2 * Zb[_2D_IDX(6,5,6,9)] - 2 * Zc[_2D_IDX(6,5,6,9)];
	GeM2x3[_2D_IDX(24,10,24,24)] = -Zb[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,7,6,9)] - Zc[_2D_IDX(6,3,6,9)] - Zb[_2D_IDX(6,3,6,9)];
	GeM2x3[_2D_IDX(24,11,24,24)] = -Zc[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,8,6,9)] - Zb[_2D_IDX(6,4,6,9)] - Zc[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(24,12,24,24)] = -Zb[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,9,6,9)] - Zc[_2D_IDX(6,6,6,9)] - Zb[_2D_IDX(6,6,6,9)];
	GeM2x3[_2D_IDX(24,16,24,24)] = 2 * Za[_2D_IDX(6,3,6,9)] - 2 * Za[_2D_IDX(6,7,6,9)];
	GeM2x3[_2D_IDX(24,17,24,24)] = -2 * Za[_2D_IDX(6,8,6,9)] + 2 * Za[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(24,18,24,24)] = 2 * Za[_2D_IDX(6,6,6,9)] - 2 * Za[_2D_IDX(6,9,6,9)];
	GeM2x3[_2D_IDX(24,19,24,24)] = -4 * Za[_2D_IDX(6,1,6,9)];
	GeM2x3[_2D_IDX(24,20,24,24)] = -4 * Za[_2D_IDX(6,2,6,9)];
	GeM2x3[_2D_IDX(24,21,24,24)] = -4 * Za[_2D_IDX(6,5,6,9)];
	GeM2x3[_2D_IDX(24,22,24,24)] = -2 * Za[_2D_IDX(6,7,6,9)] - 2 * Za[_2D_IDX(6,3,6,9)];
	GeM2x3[_2D_IDX(24,23,24,24)] = -2 * Za[_2D_IDX(6,8,6,9)] - 2 * Za[_2D_IDX(6,4,6,9)];
	GeM2x3[_2D_IDX(24,24,24,24)] = -2 * Za[_2D_IDX(6,9,6,9)] - 2 * Za[_2D_IDX(6,6,6,9)];

	/* First, solve generalized eigenvalue problem, where we are interested only in real eigenvalues */
	GENERAL_6R_DEBUG("Solving generalized eigenvalue problem.\n");
	if (geneig(GeM2x3, GeM1x3, 24, alpha_r, alpha_i, beta, eigvector))
	{
		GENERAL_6R_DEBUG("Error in solving generalized eigenvalue problem!\n");
		no_solutions = 0;
		return;
	}
	GENERAL_6R_DEBUG("Solved generalized eigenvalue problem.\n");

	/* Select valid eigenvalues */
	memset(valid_eigenval_index, EIG_VALUE_SKIP, sizeof(valid_eigenval_index));
	no_eigvalues = 0;

	for (i = 0; i < 24; i++)
	{
		if (fabs(beta[i]) < limits->eigenvalue_infinite_limit)
		{
			if (fabs(alpha_r[i]) < limits->eigenvalue_infinite_limit)
				continue; /* singular or complex */

			if (fabs(alpha_i[i]) > limits->imaginary_tol)
				continue; /* complex */

			/* infinite */
			valid_eigenval_index[no_eigvalues] = i;
			eigvalue[no_eigvalues] = 1 + (1 / limits->eigenvalue_infinite_limit);
			GENERAL_6R_DEBUG("Eigenvalue: INF\n");
			no_eigvalues++;
			continue;
		}

		if (fabs(alpha_i[i] / beta[i]) > limits->imaginary_tol)
			continue; /* complex */

		valid_eigenval_index[no_eigvalues] = i;
		eigvalue[no_eigvalues] = alpha_r[i] / beta[i];
		GENERAL_6R_DEBUG("Eigenvalue: %.5f\n", eigvalue[no_eigvalues]);
		no_eigvalues++;
	}

	GENERAL_6R_DEBUG("Total real eigenvalues: %d\n", no_eigvalues);

	/* ======================================================================
	 * STEP 3: Calculate joints 3, 4, 5
	 * ====================================================================*/

	/* Eigenvalues with algebraic multiplicity greater than 1 and considered
	 * to have geometric multiplicity also greater than 1. The eigenvectors for
	 * the same eigenvalue with geom. mult. greater than 1 are not guaranteed
	 * to be linear independent, so we cannot use a linear combination of them.
	 *
	 * Instead, we mark the eigenvalue and solve the x4, c5 and s5 using a different method.
	 */

	for (i = 0; i < no_eigvalues - 1; i++)
	{
		if (valid_eigenval_index[i] == EIG_VALUE_SKIP) /* skip */
			continue;

		for (j = i + 1; j < no_eigvalues; j++)
		{
			if ((valid_eigenval_index[j] == EIG_VALUE_SKIP)
			    || (valid_eigenval_index[j] == EIG_VALUE_ALTERNATE_METHOD)) /* skip */
				continue;

			if (fabs(eigvalue[i] - eigvalue[j]) < limits->root_tol)
			{
				/* Mark one with alt. method and the other one to skip */
				valid_eigenval_index[i] = EIG_VALUE_ALTERNATE_METHOD;
				valid_eigenval_index[j] = EIG_VALUE_ALTERNATE_METHOD;
				GENERAL_6R_DEBUG("Eigenvalue (%.5f, %.5f) might have geometric multiplicity greater than 1!\n",
				                 eigvalue[i], eigvalue[j]);
			}
		}
	}

	/* Okay, we have validated eigenvalues */
	for (i = 0; i < no_eigvalues; i++)
	{
		/* Solutions (theoretically 6 solutions max per one solution for x3) */
		sol345_t s345[6];
		double j3, s3, c3;
		int use_alternate_method = 1;

		int no_sol345 = 0;

		if (valid_eigenval_index[i] == EIG_VALUE_SKIP)
			continue;

		GENERAL_6R_DEBUG("================================================================================\n");
		GENERAL_6R_DEBUG("Solving eigenvalue %.5f\n", eigvalue[i]);

		/* Calculate s3,c3,j3 from x3 */
		if (fabs(eigvalue[i]) > (1 / limits->eigenvalue_infinite_limit))
		{
			/* pi */
			j3 = M_PI;
			s3 = 0;
			c3 = -1;
		}
		else
			x2scj(&s3, &c3, &j3, eigvalue[i]);

		GENERAL_6R_DEBUG("theta[3] = %.5f\n", j3);
		if (limits->test_joint_limits && !is_within_limit(robot, 3, j3))
		{
			GENERAL_6R_DEBUG("theta[3] is not within joint limits, skipping!\n");
			continue;
		}

		/* Check if we are parsing eigenvector */
		if (valid_eigenval_index[i] != EIG_VALUE_ALTERNATE_METHOD)
		{
			/* Dig in it, first check which one is better */
			int eigvct_idx = (fabs(eigvalue[i]) > 1) ? 12 : 0;

			/* Set s3 */
			s345[0].j3 = j3;
			s345[0].s3 = s3;
			s345[0].c3 = c3;
			no_sol345 = 1;

			/* See, if x4 is infinite */
			if (fabs(eigvector[_2D_IDX(eigvct_idx + 12, valid_eigenval_index[i] + 1, 24, 24)]) < limits->eigenvalue_infinite_limit)
			{
				/* yes */
				s345[0].j4 = M_PI;
				s345[0].s4 = 0;
				s345[0].c4 = -1;

				s345[0].s5 = eigvector[_2D_IDX(eigvct_idx + 1, valid_eigenval_index[i] + 1, 24, 24)] /
				             eigvector[_2D_IDX(eigvct_idx + 3, valid_eigenval_index[i] + 1, 24, 24)];

				s345[0].c5 = eigvector[_2D_IDX(eigvct_idx + 2, valid_eigenval_index[i] + 1, 24, 24)] /
				             eigvector[_2D_IDX(eigvct_idx + 3, valid_eigenval_index[i] + 1, 24, 24)];

				s345[0].j5 = atan2(s345[0].s5, s345[0].c5);
			}
			else
			{
				/* no */
				x2scj(&(s345[0].s4), &(s345[0].c4), &(s345[0].j4), eigvector[_2D_IDX(eigvct_idx + 9, valid_eigenval_index[i] + 1, 24, 24)] /
				      eigvector[_2D_IDX(eigvct_idx + 12, valid_eigenval_index[i] + 1, 24, 24)]);

				s345[0].s5 = eigvector[_2D_IDX(eigvct_idx + 10, valid_eigenval_index[i] + 1, 24, 24)] /
				             eigvector[_2D_IDX(eigvct_idx + 12, valid_eigenval_index[i] + 1, 24, 24)];

				s345[0].c5 = eigvector[_2D_IDX(eigvct_idx + 11, valid_eigenval_index[i] + 1, 24, 24)] /
				             eigvector[_2D_IDX(eigvct_idx + 12, valid_eigenval_index[i] + 1, 24, 24)];

				s345[0].j5 = atan2(s345[0].s5, s345[0].c5);
			}

			/* Check validity, otherwise using alternate method is necessary */
			if ((fabs(eigvector[_2D_IDX(eigvct_idx + 6, valid_eigenval_index[i] + 1, 24, 24)] -
							  eigvector[_2D_IDX(eigvct_idx + 9, valid_eigenval_index[i] + 1, 24, 24)] * eigvector[_2D_IDX(eigvct_idx + 9, valid_eigenval_index[i] + 1, 24, 24)]) < limits->eigenvalue_safety)
					&& (sin_cos_valid(s345[0].s5, s345[0].c5, limits->eigenvalue_safety)) )
			{
				use_alternate_method = 0;
				GENERAL_6R_DEBUG("Only one solution for theta[4] and theta[5]\n");
			}
			else
				GENERAL_6R_DEBUG("Failed resolving theta[4] and theta[5] from the eigenvector!\n");
		}

		if (use_alternate_method)
		{
			/* For generalized eigenvalue problem to see x4 */
			double GeM1x4[6*6], GeM2x4[6*6];
			double alpha_r4[6], alpha_i4[6], beta4[6];
			double eigvals4[6];
			int no_eigvalues4 = 0;

			GENERAL_6R_DEBUG("Using alternative method to resolve theta[4] and theta[5].\n");

			memset(GeM1x4, 0, sizeof(GeM1x4));
			memset(GeM2x4, 0, sizeof(GeM2x4));

			GeM1x4[_2D_IDX(1,1,6,6)] = 1;
			GeM1x4[_2D_IDX(2,2,6,6)] = 1;
			GeM1x4[_2D_IDX(3,3,6,6)] = 1;
			GeM1x4[_2D_IDX(4,4,6,6)] = Zc[_2D_IDX(1,7,6,9)] - c3 * Zb[_2D_IDX(1,3,6,9)] + s3 * Za[_2D_IDX(1,7,6,9)] + c3 * Zb[_2D_IDX(1,7,6,9)] - s3 * Za[_2D_IDX(1,3,6,9)] - Zc[_2D_IDX(1,3,6,9)];
			GeM1x4[_2D_IDX(4,5,6,6)] = -s3 * Za[_2D_IDX(1,4,6,9)] - c3 * Zb[_2D_IDX(1,4,6,9)] + s3 * Za[_2D_IDX(1,8,6,9)] + c3 * Zb[_2D_IDX(1,8,6,9)] + Zc[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,4,6,9)];
			GeM1x4[_2D_IDX(4,6,6,6)] = -s3 * Za[_2D_IDX(1,6,6,9)] + s3 * Za[_2D_IDX(1,9,6,9)] + c3 * Zb[_2D_IDX(1,9,6,9)] - c3 * Zb[_2D_IDX(1,6,6,9)] - Zc[_2D_IDX(1,6,6,9)] + Zc[_2D_IDX(1,9,6,9)];
			GeM1x4[_2D_IDX(5,4,6,6)] = Zc[_2D_IDX(2,7,6,9)] - c3 * Zb[_2D_IDX(2,3,6,9)] + s3 * Za[_2D_IDX(2,7,6,9)] + c3 * Zb[_2D_IDX(2,7,6,9)] - s3 * Za[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,3,6,9)];
			GeM1x4[_2D_IDX(5,5,6,6)] = -s3 * Za[_2D_IDX(2,4,6,9)] - c3 * Zb[_2D_IDX(2,4,6,9)] + s3 * Za[_2D_IDX(2,8,6,9)] + c3 * Zb[_2D_IDX(2,8,6,9)] + Zc[_2D_IDX(2,8,6,9)] - Zc[_2D_IDX(2,4,6,9)];
			GeM1x4[_2D_IDX(5,6,6,6)] = -s3 * Za[_2D_IDX(2,6,6,9)] + s3 * Za[_2D_IDX(2,9,6,9)] + c3 * Zb[_2D_IDX(2,9,6,9)] - c3 * Zb[_2D_IDX(2,6,6,9)] - Zc[_2D_IDX(2,6,6,9)] + Zc[_2D_IDX(2,9,6,9)];
			GeM1x4[_2D_IDX(6,4,6,6)] = Zc[_2D_IDX(3,7,6,9)] - c3 * Zb[_2D_IDX(3,3,6,9)] + s3 * Za[_2D_IDX(3,7,6,9)] + c3 * Zb[_2D_IDX(3,7,6,9)] - s3 * Za[_2D_IDX(3,3,6,9)] - Zc[_2D_IDX(3,3,6,9)];
			GeM1x4[_2D_IDX(6,5,6,6)] = -s3 * Za[_2D_IDX(3,4,6,9)] - c3 * Zb[_2D_IDX(3,4,6,9)] + s3 * Za[_2D_IDX(3,8,6,9)] + c3 * Zb[_2D_IDX(3,8,6,9)] + Zc[_2D_IDX(3,8,6,9)] - Zc[_2D_IDX(3,4,6,9)];
			GeM1x4[_2D_IDX(6,6,6,6)] = -s3 * Za[_2D_IDX(3,6,6,9)] + s3 * Za[_2D_IDX(3,9,6,9)] + c3 * Zb[_2D_IDX(3,9,6,9)] - c3 * Zb[_2D_IDX(3,6,6,9)] - Zc[_2D_IDX(3,6,6,9)] + Zc[_2D_IDX(3,9,6,9)];

			GeM2x4[_2D_IDX(1,4,6,6)] = 1;
			GeM2x4[_2D_IDX(2,5,6,6)] = 1;
			GeM2x4[_2D_IDX(3,6,6,6)] = 1;
			GeM2x4[_2D_IDX(4,1,6,6)] = -c3 * Zb[_2D_IDX(1,7,6,9)] - Zc[_2D_IDX(1,3,6,9)] - c3 * Zb[_2D_IDX(1,3,6,9)] - Zc[_2D_IDX(1,7,6,9)] - s3 * Za[_2D_IDX(1,7,6,9)] - s3 * Za[_2D_IDX(1,3,6,9)];
			GeM2x4[_2D_IDX(4,2,6,6)] = -c3 * Zb[_2D_IDX(1,4,6,9)] - s3 * Za[_2D_IDX(1,8,6,9)] - Zc[_2D_IDX(1,4,6,9)] - s3 * Za[_2D_IDX(1,4,6,9)] - Zc[_2D_IDX(1,8,6,9)] - c3 * Zb[_2D_IDX(1,8,6,9)];
			GeM2x4[_2D_IDX(4,3,6,6)] = -Zc[_2D_IDX(1,9,6,9)] - s3 * Za[_2D_IDX(1,6,6,9)] - s3 * Za[_2D_IDX(1,9,6,9)] - c3 * Zb[_2D_IDX(1,6,6,9)] - c3 * Zb[_2D_IDX(1,9,6,9)] - Zc[_2D_IDX(1,6,6,9)];
			GeM2x4[_2D_IDX(4,4,6,6)] = -2 * c3 * Zb[_2D_IDX(1,1,6,9)] - 2 * s3 * Za[_2D_IDX(1,1,6,9)] - 2 * Zc[_2D_IDX(1,1,6,9)];
			GeM2x4[_2D_IDX(4,5,6,6)] = -2 * c3 * Zb[_2D_IDX(1,2,6,9)] - 2 * s3 * Za[_2D_IDX(1,2,6,9)] - 2 * Zc[_2D_IDX(1,2,6,9)];
			GeM2x4[_2D_IDX(4,6,6,6)] = -2 * s3 * Za[_2D_IDX(1,5,6,9)] - 2 * c3 * Zb[_2D_IDX(1,5,6,9)] - 2 * Zc[_2D_IDX(1,5,6,9)];
			GeM2x4[_2D_IDX(5,1,6,6)] = -c3 * Zb[_2D_IDX(2,7,6,9)] - Zc[_2D_IDX(2,3,6,9)] - c3 * Zb[_2D_IDX(2,3,6,9)] - Zc[_2D_IDX(2,7,6,9)] - s3 * Za[_2D_IDX(2,7,6,9)] - s3 * Za[_2D_IDX(2,3,6,9)];
			GeM2x4[_2D_IDX(5,2,6,6)] = -c3 * Zb[_2D_IDX(2,4,6,9)] - s3 * Za[_2D_IDX(2,8,6,9)] - Zc[_2D_IDX(2,4,6,9)] - s3 * Za[_2D_IDX(2,4,6,9)] - Zc[_2D_IDX(2,8,6,9)] - c3 * Zb[_2D_IDX(2,8,6,9)];
			GeM2x4[_2D_IDX(5,3,6,6)] = -Zc[_2D_IDX(2,9,6,9)] - s3 * Za[_2D_IDX(2,6,6,9)] - s3 * Za[_2D_IDX(2,9,6,9)] - c3 * Zb[_2D_IDX(2,6,6,9)] - c3 * Zb[_2D_IDX(2,9,6,9)] - Zc[_2D_IDX(2,6,6,9)];
			GeM2x4[_2D_IDX(5,4,6,6)] = -2 * c3 * Zb[_2D_IDX(2,1,6,9)] - 2 * s3 * Za[_2D_IDX(2,1,6,9)] - 2 * Zc[_2D_IDX(2,1,6,9)];
			GeM2x4[_2D_IDX(5,5,6,6)] = -2 * c3 * Zb[_2D_IDX(2,2,6,9)] - 2 * s3 * Za[_2D_IDX(2,2,6,9)] - 2 * Zc[_2D_IDX(2,2,6,9)];
			GeM2x4[_2D_IDX(5,6,6,6)] = -2 * s3 * Za[_2D_IDX(2,5,6,9)] - 2 * c3 * Zb[_2D_IDX(2,5,6,9)] - 2 * Zc[_2D_IDX(2,5,6,9)];
			GeM2x4[_2D_IDX(6,1,6,6)] = -c3 * Zb[_2D_IDX(3,7,6,9)] - Zc[_2D_IDX(3,3,6,9)] - c3 * Zb[_2D_IDX(3,3,6,9)] - Zc[_2D_IDX(3,7,6,9)] - s3 * Za[_2D_IDX(3,7,6,9)] - s3 * Za[_2D_IDX(3,3,6,9)];
			GeM2x4[_2D_IDX(6,2,6,6)] = -c3 * Zb[_2D_IDX(3,4,6,9)] - s3 * Za[_2D_IDX(3,8,6,9)] - Zc[_2D_IDX(3,4,6,9)] - s3 * Za[_2D_IDX(3,4,6,9)] - Zc[_2D_IDX(3,8,6,9)] - c3 * Zb[_2D_IDX(3,8,6,9)];
			GeM2x4[_2D_IDX(6,3,6,6)] = -Zc[_2D_IDX(3,9,6,9)] - s3 * Za[_2D_IDX(3,6,6,9)] - s3 * Za[_2D_IDX(3,9,6,9)] - c3 * Zb[_2D_IDX(3,6,6,9)] - c3 * Zb[_2D_IDX(3,9,6,9)] - Zc[_2D_IDX(3,6,6,9)];
			GeM2x4[_2D_IDX(6,4,6,6)] = -2 * c3 * Zb[_2D_IDX(3,1,6,9)] - 2 * s3 * Za[_2D_IDX(3,1,6,9)] - 2 * Zc[_2D_IDX(3,1,6,9)];
			GeM2x4[_2D_IDX(6,5,6,6)] = -2 * c3 * Zb[_2D_IDX(3,2,6,9)] - 2 * s3 * Za[_2D_IDX(3,2,6,9)] - 2 * Zc[_2D_IDX(3,2,6,9)];
			GeM2x4[_2D_IDX(6,6,6,6)] = -2 * s3 * Za[_2D_IDX(3,5,6,9)] - 2 * c3 * Zb[_2D_IDX(3,5,6,9)] - 2 * Zc[_2D_IDX(3,5,6,9)];

			GENERAL_6R_DEBUG("Solving generalized eigenvalue problem.\n");
			if (geneig(GeM2x4, GeM1x4, 6, alpha_r4, alpha_i4, beta4, NULL))
			{
				GENERAL_6R_DEBUG("Error in solving generalized eigenvalue problem!\n");
				continue;
			}
			GENERAL_6R_DEBUG("Solved generalized eigenvalue problem.\n");

			/* Make sure there are no duplicities (we don't need to know if an eigenvalue has greater geom. multiplicity than 1 now */
			for (j = 0; j < 6; j++)
			{
				int add = 1;
				double eig4;

				if (fabs(beta4[j]) < limits->eigenvalue_infinite_limit)
				{
					if (fabs(alpha_r4[j]) < limits->eigenvalue_infinite_limit)
						continue; /* singular or complex */

					if (fabs(alpha_i4[j]) > limits->imaginary_tol)
						continue; /* complex */

					/* infinite */
					eig4 = (1 + (1 / limits->eigenvalue_infinite_limit));
					for (k = 0; k < no_eigvalues4; k++)
					{
						if (fabs(eigvals4[k] - eig4) <  limits->root_tol)
						{
							add = 0;
							break;
						}
					}

					if (!add)
						continue;

					eigvals4[no_eigvalues4] = eig4;
					GENERAL_6R_DEBUG("Eigenvalue: INF\n");
					no_eigvalues4++;
					continue;
				}

				if (fabs(alpha_i4[j] / beta4[j]) > limits->imaginary_tol)
					continue; /* complex */

				eig4 = alpha_r4[j] / beta4[j];
				for (k = 0; k < no_eigvalues4; k++)
				{
					if (fabs(eigvals4[k] - eig4) <  limits->root_tol)
					{
						add = 0;
						break;
					}
				}

				if (!add)
					continue;

				eigvals4[no_eigvalues4] = eig4;
				GENERAL_6R_DEBUG("Eigenvalue: %.5f\n", eigvals4[no_eigvalues4]);
				no_eigvalues4++;
			}

			for (j = 0; j < no_eigvalues4; j++)
			{
				/* Joint 4 variables */
				double j4, s4, c4;

				/* Matrices for solving joint 5 */
				double M5l[6*2], M5r[6];

				/* Permutation for LU decomposition of M5l and M6l */
				int Pm5[6*1];

				double ys, yc;
				double s5, c5, j5;

				GENERAL_6R_DEBUG("--------------------------------------------------------------------------------\n");

				if (fabs(eigvals4[j]) > (1 / limits->eigenvalue_infinite_limit))
				{
					/* infinite */
					j4 = M_PI;
					s4 = 0;
					c4 = -1;
				}
				else
					x2scj(&s4, &c4, &j4, eigvals4[j]);

				GENERAL_6R_DEBUG("theta[4] = %.5f\n", j4);
				if (limits->test_joint_limits && !is_within_limit(robot, 4, j4))
				{
					GENERAL_6R_DEBUG("theta[4] is not within joint limits, skipping!\n");
					continue;
				}

				/* Calculate s5, c5, j5
				 * This joint is concrete, so there can be only one solution.
				 */
				memset(M5l, 0, sizeof(M5l));
				memset(M5r, 0, sizeof(M5r));

				M5l[_2D_IDX(1,1,6,2)] = c3 * Zb[_2D_IDX(1,7,6,9)] + s4 * Zc[_2D_IDX(1,1,6,9)] + s4 * c3 * Zb[_2D_IDX(1,1,6,9)] + s3 * Za[_2D_IDX(1,7,6,9)] + c4 * s3 * Za[_2D_IDX(1,3,6,9)] + c4 * c3 * Zb[_2D_IDX(1,3,6,9)] + c4 * Zc[_2D_IDX(1,3,6,9)] + Zc[_2D_IDX(1,7,6,9)] + s4 * s3 * Za[_2D_IDX(1,1,6,9)];
				M5l[_2D_IDX(1,2,6,2)] = Zc[_2D_IDX(1,8,6,9)] + s4 * Zc[_2D_IDX(1,2,6,9)] + s4 * s3 * Za[_2D_IDX(1,2,6,9)] + s4 * c3 * Zb[_2D_IDX(1,2,6,9)] + c4 * Zc[_2D_IDX(1,4,6,9)] + s3 * Za[_2D_IDX(1,8,6,9)] + c3 * Zb[_2D_IDX(1,8,6,9)] + c4 * c3 * Zb[_2D_IDX(1,4,6,9)] + c4 * s3 * Za[_2D_IDX(1,4,6,9)];
				M5l[_2D_IDX(2,1,6,2)] = c3 * Zb[_2D_IDX(2,7,6,9)] + s4 * Zc[_2D_IDX(2,1,6,9)] + s4 * c3 * Zb[_2D_IDX(2,1,6,9)] + s3 * Za[_2D_IDX(2,7,6,9)] + c4 * s3 * Za[_2D_IDX(2,3,6,9)] + c4 * c3 * Zb[_2D_IDX(2,3,6,9)] + c4 * Zc[_2D_IDX(2,3,6,9)] + Zc[_2D_IDX(2,7,6,9)] + s4 * s3 * Za[_2D_IDX(2,1,6,9)];
				M5l[_2D_IDX(2,2,6,2)] = Zc[_2D_IDX(2,8,6,9)] + s4 * Zc[_2D_IDX(2,2,6,9)] + s4 * s3 * Za[_2D_IDX(2,2,6,9)] + s4 * c3 * Zb[_2D_IDX(2,2,6,9)] + c4 * Zc[_2D_IDX(2,4,6,9)] + s3 * Za[_2D_IDX(2,8,6,9)] + c3 * Zb[_2D_IDX(2,8,6,9)] + c4 * c3 * Zb[_2D_IDX(2,4,6,9)] + c4 * s3 * Za[_2D_IDX(2,4,6,9)];
				M5l[_2D_IDX(3,1,6,2)] = c3 * Zb[_2D_IDX(3,7,6,9)] + s4 * Zc[_2D_IDX(3,1,6,9)] + s4 * c3 * Zb[_2D_IDX(3,1,6,9)] + s3 * Za[_2D_IDX(3,7,6,9)] + c4 * s3 * Za[_2D_IDX(3,3,6,9)] + c4 * c3 * Zb[_2D_IDX(3,3,6,9)] + c4 * Zc[_2D_IDX(3,3,6,9)] + Zc[_2D_IDX(3,7,6,9)] + s4 * s3 * Za[_2D_IDX(3,1,6,9)];
				M5l[_2D_IDX(3,2,6,2)] = Zc[_2D_IDX(3,8,6,9)] + s4 * Zc[_2D_IDX(3,2,6,9)] + s4 * s3 * Za[_2D_IDX(3,2,6,9)] + s4 * c3 * Zb[_2D_IDX(3,2,6,9)] + c4 * Zc[_2D_IDX(3,4,6,9)] + s3 * Za[_2D_IDX(3,8,6,9)] + c3 * Zb[_2D_IDX(3,8,6,9)] + c4 * c3 * Zb[_2D_IDX(3,4,6,9)] + c4 * s3 * Za[_2D_IDX(3,4,6,9)];
				M5l[_2D_IDX(4,1,6,2)] = c3 * Zb[_2D_IDX(4,7,6,9)] + s4 * Zc[_2D_IDX(4,1,6,9)] + s4 * c3 * Zb[_2D_IDX(4,1,6,9)] + s3 * Za[_2D_IDX(4,7,6,9)] + c4 * s3 * Za[_2D_IDX(4,3,6,9)] + c4 * c3 * Zb[_2D_IDX(4,3,6,9)] + c4 * Zc[_2D_IDX(4,3,6,9)] + Zc[_2D_IDX(4,7,6,9)] + s4 * s3 * Za[_2D_IDX(4,1,6,9)];
				M5l[_2D_IDX(4,2,6,2)] = Zc[_2D_IDX(4,8,6,9)] + s4 * Zc[_2D_IDX(4,2,6,9)] + s4 * s3 * Za[_2D_IDX(4,2,6,9)] + s4 * c3 * Zb[_2D_IDX(4,2,6,9)] + c4 * Zc[_2D_IDX(4,4,6,9)] + s3 * Za[_2D_IDX(4,8,6,9)] + c3 * Zb[_2D_IDX(4,8,6,9)] + c4 * c3 * Zb[_2D_IDX(4,4,6,9)] + c4 * s3 * Za[_2D_IDX(4,4,6,9)];
				M5l[_2D_IDX(5,1,6,2)] = s4 * c3 * Zb[_2D_IDX(5,1,6,9)] + c3 * Zb[_2D_IDX(5,7,6,9)] + s4 * Zc[_2D_IDX(5,1,6,9)] + Zc[_2D_IDX(5,7,6,9)] + c4 * s3 * Za[_2D_IDX(5,3,6,9)] + c4 * c3 * Zb[_2D_IDX(5,3,6,9)] + s4 * s3 * Za[_2D_IDX(5,1,6,9)] + c4 * Zc[_2D_IDX(5,3,6,9)] + s3 * Za[_2D_IDX(5,7,6,9)];
				M5l[_2D_IDX(5,2,6,2)] = c3 * Zb[_2D_IDX(5,8,6,9)] + Zc[_2D_IDX(5,8,6,9)] + s4 * s3 * Za[_2D_IDX(5,2,6,9)] + s4 * c3 * Zb[_2D_IDX(5,2,6,9)] + c4 * Zc[_2D_IDX(5,4,6,9)] + s3 * Za[_2D_IDX(5,8,6,9)] + s4 * Zc[_2D_IDX(5,2,6,9)] + c4 * c3 * Zb[_2D_IDX(5,4,6,9)] + c4 * s3 * Za[_2D_IDX(5,4,6,9)];
				M5l[_2D_IDX(6,1,6,2)] = s4 * c3 * Zb[_2D_IDX(6,1,6,9)] + c3 * Zb[_2D_IDX(6,7,6,9)] + s4 * Zc[_2D_IDX(6,1,6,9)] + Zc[_2D_IDX(6,7,6,9)] + c4 * s3 * Za[_2D_IDX(6,3,6,9)] + c4 * c3 * Zb[_2D_IDX(6,3,6,9)] + s4 * s3 * Za[_2D_IDX(6,1,6,9)] + c4 * Zc[_2D_IDX(6,3,6,9)] + s3 * Za[_2D_IDX(6,7,6,9)];
				M5l[_2D_IDX(6,2,6,2)] = c3 * Zb[_2D_IDX(6,8,6,9)] + Zc[_2D_IDX(6,8,6,9)] + s4 * s3 * Za[_2D_IDX(6,2,6,9)] + s4 * c3 * Zb[_2D_IDX(6,2,6,9)] + c4 * Zc[_2D_IDX(6,4,6,9)] + s3 * Za[_2D_IDX(6,8,6,9)] + s4 * Zc[_2D_IDX(6,2,6,9)] + c4 * c3 * Zb[_2D_IDX(6,4,6,9)] + c4 * s3 * Za[_2D_IDX(6,4,6,9)];

				M5r[0] = -Zc[_2D_IDX(1,9,6,9)] - s3 * Za[_2D_IDX(1,9,6,9)] - c3 * Zb[_2D_IDX(1,9,6,9)] - s4 * Zc[_2D_IDX(1,5,6,9)] - c4 * Zc[_2D_IDX(1,6,6,9)] - s4 * s3 * Za[_2D_IDX(1,5,6,9)] - s4 * c3 * Zb[_2D_IDX(1,5,6,9)] - c4 * s3 * Za[_2D_IDX(1,6,6,9)] - c4 * c3 * Zb[_2D_IDX(1,6,6,9)];
				M5r[1] = -Zc[_2D_IDX(2,9,6,9)] - s3 * Za[_2D_IDX(2,9,6,9)] - c3 * Zb[_2D_IDX(2,9,6,9)] - s4 * Zc[_2D_IDX(2,5,6,9)] - c4 * Zc[_2D_IDX(2,6,6,9)] - s4 * s3 * Za[_2D_IDX(2,5,6,9)] - s4 * c3 * Zb[_2D_IDX(2,5,6,9)] - c4 * s3 * Za[_2D_IDX(2,6,6,9)] - c4 * c3 * Zb[_2D_IDX(2,6,6,9)];
				M5r[2] = -Zc[_2D_IDX(3,9,6,9)] - s3 * Za[_2D_IDX(3,9,6,9)] - c3 * Zb[_2D_IDX(3,9,6,9)] - s4 * Zc[_2D_IDX(3,5,6,9)] - c4 * Zc[_2D_IDX(3,6,6,9)] - s4 * s3 * Za[_2D_IDX(3,5,6,9)] - s4 * c3 * Zb[_2D_IDX(3,5,6,9)] - c4 * s3 * Za[_2D_IDX(3,6,6,9)] - c4 * c3 * Zb[_2D_IDX(3,6,6,9)];
				M5r[3] = -Zc[_2D_IDX(4,9,6,9)] - s3 * Za[_2D_IDX(4,9,6,9)] - c3 * Zb[_2D_IDX(4,9,6,9)] - s4 * Zc[_2D_IDX(4,5,6,9)] - c4 * Zc[_2D_IDX(4,6,6,9)] - s4 * s3 * Za[_2D_IDX(4,5,6,9)] - s4 * c3 * Zb[_2D_IDX(4,5,6,9)] - c4 * s3 * Za[_2D_IDX(4,6,6,9)] - c4 * c3 * Zb[_2D_IDX(4,6,6,9)];
				M5r[4] = -Zc[_2D_IDX(5,9,6,9)] - s3 * Za[_2D_IDX(5,9,6,9)] - c3 * Zb[_2D_IDX(5,9,6,9)] - s4 * c3 * Zb[_2D_IDX(5,5,6,9)] - s4 * Zc[_2D_IDX(5,5,6,9)] - c4 * Zc[_2D_IDX(5,6,6,9)] - s4 * s3 * Za[_2D_IDX(5,5,6,9)] - c4 * s3 * Za[_2D_IDX(5,6,6,9)] - c4 * c3 * Zb[_2D_IDX(5,6,6,9)];
				M5r[5] = -Zc[_2D_IDX(6,9,6,9)] - s3 * Za[_2D_IDX(6,9,6,9)] - c3 * Zb[_2D_IDX(6,9,6,9)] - s4 * c3 * Zb[_2D_IDX(6,5,6,9)] - s4 * Zc[_2D_IDX(6,5,6,9)] - c4 * Zc[_2D_IDX(6,6,6,9)] - s4 * s3 * Za[_2D_IDX(6,5,6,9)] - c4 * s3 * Za[_2D_IDX(6,6,6,9)] - c4 * c3 * Zb[_2D_IDX(6,6,6,9)];

				/* LU decomposition */
				GENERAL_6R_DEBUG("Decomposing M5l matrix using LU.\n");
				if (lu_quick(M5l, Pm5, 6, 2))
				{
					GENERAL_6R_DEBUG("Error in LU decomposition!\n");
					continue;
				}
				GENERAL_6R_DEBUG("Decomposed M5l matrix using LU.\n");

				/* Check rank - it must be full now */
				if (fabs(M5l[_2D_IDX(1,1,6,2)]) < limits->perturbation ||
						fabs(M5l[_2D_IDX(2,2,6,2)]) < limits->perturbation)
				{
					GENERAL_6R_DEBUG("theta[5] cannot be parametrized, discarding!\n");
					continue;
				}

				/* Solve L * y = Pm6 * M6r */
				ys = M5r[Pm5[0]];
				yc = M5r[Pm5[1]] - ys * M5l[_2D_IDX(2,1,6,2)];

				/* Solve U * x = y */
				c5 =  yc / M5l[_2D_IDX(2,2,6,2)];
				s5 = (ys - c5 * M5l[_2D_IDX(1,2,6,2)]) / M5l[_2D_IDX(1,1,6,2)];

				if (!sin_cos_valid(s5, c5, SIN_COS_UNOPTIMIZED_TOLERANCE))
				{
					GENERAL_6R_DEBUG("theta[5] is invalid, discarding (s5 = %.5f, c5 = %.5f)!\n", s5, c5);
					continue;
				}

				j5 = base_angle(atan2(s5, c5));
				GENERAL_6R_DEBUG("theta[5] = %.5f\n", j5);
				if (limits->test_joint_limits && !is_within_limit(robot, 5, j5))
				{
					GENERAL_6R_DEBUG("theta[5] is not within joint limits, skipping!\n");
					continue;
				}

				s345[no_sol345].j3 = j3;
				s345[no_sol345].s3 = s3;
				s345[no_sol345].c3 = c3;

				s345[no_sol345].j4 = j4;
				s345[no_sol345].s4 = s4;
				s345[no_sol345].c4 = c4;

				s345[no_sol345].j5 = j5;
				s345[no_sol345].s5 = s5;
				s345[no_sol345].c5 = c5;

				no_sol345++;
			}

			sol_alternate_method = 1;
		}
		else
			sol_alternate_method = 0;

		/* OK - now we have j3 - j5, calculate j1, j2 from ME matrices */
		for (j = 0; j < no_sol345; j++)
		{
			double s2[2], c2[2], j2[2];
			int f2;
			int no_sol2 = 1;
			int sol_singular = 0;
			int newton_steps;

			GENERAL_6R_DEBUG("--------------------------------------------------------------------------------\n");
			GENERAL_6R_DEBUG("Evaluating joints 3 - 5 solution %d: theta[3] = %.5f, theta[4] = %.5f, theta[5] = %.5f\n",
							         j, s345[j].j3, s345[j].j4, s345[j].j5);

			if (limits->test_joint_limits && !is_within_limit(robot, 4, s345[j].j4))
			{
				GENERAL_6R_DEBUG("theta[4] is not within joint limits, skipping!\n");
				continue;
			}

			if (limits->test_joint_limits && !is_within_limit(robot, 5, s345[j].j5))
			{
				GENERAL_6R_DEBUG("theta[5] is not within joint limits, skipping!\n");
				continue;
			}

			/* First, optimize it j3 - j5 */
			GENERAL_6R_DEBUG("Running Newton optimization\n");
			newton_steps = newton_optimization_345(Za, Zb, Zc, &(s345[j]), limits);
			GENERAL_6R_DEBUG("Finished Newton optimization (%d steps)\n", newton_steps);
			GENERAL_6R_DEBUG("Newton optimized to theta[3] = %.5f, theta[4] = %.5f, theta[5] = %.5f\n",
							         s345[j].j3, s345[j].j4, s345[j].j5);

			if (flags && newton_steps > 0)
				sol_newton_optimized = 1;
			else
				sol_newton_optimized = 0;

			/* Fill LP vector */
			LP[0] = s345[j].s4 * s345[j].s5;
			LP[1] = s345[j].s4 * s345[j].c5;
			LP[2] = s345[j].c4 * s345[j].s5;
			LP[3] = s345[j].c4 * s345[j].c5;
			LP[4] = s345[j].s4;
			LP[5] = s345[j].c4;
			LP[6] = s345[j].s5;
			LP[7] = s345[j].c5;
			LP[8] = 1.0;

			/* ======================================================================
			 * STEP 4: Compute joint 2
			 * ====================================================================*/

			/* Calculate ME matrix from columns (3,4) */
			for (l = 1; l <= 2; l++)
				for (m = 1; m <= 9; m++)
					MECalc[_2D_IDX(l,m,2,9)] = MEa[_2D_IDX(l+2,m,4,9)] * s345[j].s3 + MEb[_2D_IDX(l+2,m,4,9)] * s345[j].c3 +
																		 MEc[_2D_IDX(l+2,m,4,9)];

			/* Calculate the left side of the equation */
			matrix_mult(LB2, MECalc, 2, 9, LP, 9, 1);

			/* Compute the equation */
			sin_cos_equation_triangle(&(s2[0]), &(c2[0]), &(j2[0]), &f2, RB2, LB2, limits, 1, 1);
			GENERAL_6R_DEBUG("Resolved theta[2]\n");

			if (f2 == SIN_COS_NO_SOLUTION)
			{
				GENERAL_6R_DEBUG("No solution for theta[2] (s2 = %.5f, c2 = %.5f)!\n", s2[0], c2[0]);
				continue;
			}

			/* Generate the solutions */
			switch(f2)
			{
				case SIN_COS_FIXED_COS:
					s2[1] = -s2[0];
					c2[1] = c2[0];
					j2[1] = base_angle(M_PI - j2[0]);
					no_sol2 = 2;
					GENERAL_6R_DEBUG("2 solutions for theta[2] (sharing COS)!\n");
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[0]);
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[1]);
					break;

				case SIN_COS_FIXED_SIN:
					s2[1] = s2[0];
					c2[1] = -c2[0];
					j2[1] = -j2[0];
					no_sol2 = 2;
					GENERAL_6R_DEBUG("2 solutions for theta[2] (sharing SIN)!\n");
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[0]);
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[1]);
					break;

				case SIN_COS_ZERO_RANK:
					sol_singular = 1;
					GENERAL_6R_DEBUG("Parametrized solution for theta[2].\n");
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[0]);
					break;

				default:
					GENERAL_6R_DEBUG("1 solution for theta[2].\n");
					GENERAL_6R_DEBUG("theta[2] = %.5f\n", j2[0]);
					break;
			}

			/* ======================================================================
			 * STEP 5: Compute joint 1
			 * ====================================================================*/

			/* For every solution of j2, calculate j1
			 * but first calculate ME matrix (columns 1,2)
			 * as it is same for both
			 */
			for (l = 1; l <= 2; l++)
				for (m = 1; m <= 9; m++)
					MECalc[_2D_IDX(l,m,2,9)] = MEa[_2D_IDX(l,m,4,9)] * s345[j].s3 + MEb[_2D_IDX(l,m,4,9)] * s345[j].c3 +
																		 MEc[_2D_IDX(l,m,4,9)];

			/* Calculate the left side of the equation
			 * Use again LB2 here, then update it with the current s2,c2 to LB1
			 */
			matrix_mult(LB2, MECalc, 2, 9, LP, 9, 1);

			for (k = 0; k < no_sol2; k++)
			{
				double s1[2], c1[2], j1[2];
				int f1;
				int no_sol1 = 1;

				/* First, check if the theta2 we're using now is legitimate */
				if (limits->test_joint_limits && !is_within_limit(robot, 2, j2[k]))
				{
					GENERAL_6R_DEBUG("theta[2] is not within joint limits, skipping!\n");
					continue;
				}

				/* Calculate LB1 */
				LB1[0] = LB2[0] - R[_2D_IDX(5,7,14,8)]*s2[k] - R[_2D_IDX(5,8,14,8)]*c2[k];
				LB1[1] = LB2[1] - R[_2D_IDX(6,7,14,8)]*s2[k] - R[_2D_IDX(6,8,14,8)]*c2[k];

				/* Compute the equation */
				sin_cos_equation_triangle(&(s1[0]), &(c1[0]), &(j1[0]), &f1, RB1, LB1, limits, 1, 1);
				GENERAL_6R_DEBUG("Resolved theta[1]\n");

				if (f1 == SIN_COS_NO_SOLUTION)
				{
					GENERAL_6R_DEBUG("No solution for theta[1] (s1 = %.5f, c1 = %.5f)!\n", s1[0], c1[0]);
					continue;
				}

				/* Generate the solutions */
				switch(f1)
				{
					case SIN_COS_FIXED_COS:
						s1[1] = -s1[0];
						c1[1] = c1[0];
						j1[1] = base_angle(M_PI - j1[0]);
						no_sol1 = 2;
						GENERAL_6R_DEBUG("2 solutions for theta[1] (sharing COS)!\n");
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[0]);
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[1]);
						break;

					case SIN_COS_FIXED_SIN:
						s1[1] = s1[0];
						c1[1] = -c1[0];
						j1[1] = -j1[0];
						no_sol1 = 2;
						GENERAL_6R_DEBUG("2 solutions for theta[1] (sharing SIN)!\n");
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[0]);
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[1]);
						break;

					case SIN_COS_ZERO_RANK:
						sol_singular = 1;
						GENERAL_6R_DEBUG("Parametrized solution for theta[1].\n");
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[0]);
						break;

					default:
						GENERAL_6R_DEBUG("1 solution for theta[1].\n");
						GENERAL_6R_DEBUG("theta[1] = %.5f\n", j1[0]);
						break;
				}

				/* ======================================================================
				 * STEP 6: Compute joint 6
				 * ====================================================================*/

				for (l = 0; l < no_sol1; l++)
				{
					/* Matrices for calculation of the sixth joint */
					double M6l[6*2], M6r[6];

					/* Permutation for LU decomposition of M6l */
					int Pm6[6*1];

					double ys, yc;
					double s6, c6, j6;
					int duplication = 0;

					/* First, check if the theta1 we're using now is legitimate */
					if (limits->test_joint_limits && !is_within_limit(robot, 1, j1[l]))
					{
						GENERAL_6R_DEBUG("theta[1] is not within joint limits, skipping!\n");
						continue;
					}

					/* Just a reminder:
					 * 'l' iterates over s1,c1
					 * 'k' iterates over s2,c2
					 * 'j' iterates over set of solutions for joints 3,4,5
					 * 'i' iterates over eigenvalues
					 */
					memset(M6l, 0, sizeof(M6l));
					memset(M6r, 0, sizeof(M6r));

					robot->form_j6_ikt_matrices(robot, M6l, M6r, MhV, s1[l], c1[l], s2[k], c2[k], s345[j].s3, s345[j].c3,
					                            s345[j].s4, s345[j].c4, s345[j].s5, s345[j].c5);

					/* LU decomposition */
					GENERAL_6R_DEBUG("Decomposing M6l matrix using LU.\n");
					if (lu_quick(M6l, Pm6, 6, 2))
					{
						GENERAL_6R_DEBUG("Error in LU decomposition!\n");
						continue;
					}
					GENERAL_6R_DEBUG("Decomposed M6l matrix using LU.\n");

					/* Check rank - it must be full now */
					if (fabs(M6l[_2D_IDX(1,1,6,2)]) < limits->perturbation ||
							fabs(M6l[_2D_IDX(2,2,6,2)]) < limits->perturbation)
					{
						GENERAL_6R_DEBUG("theta[6] cannot be parametrized, discarding!\n");
						continue;
					}

					/* Solve L * y = Pm6 * M6r */
					ys = M6r[Pm6[0]];
					yc = M6r[Pm6[1]] - ys * M6l[_2D_IDX(2,1,6,2)];

					/* Solve U * x = y */
					c6 =  yc / M6l[_2D_IDX(2,2,6,2)];
					s6 = (ys - c6 * M6l[_2D_IDX(1,2,6,2)]) / M6l[_2D_IDX(1,1,6,2)];

					if (!sin_cos_valid(s6, c6, limits->perturbation))
					{
						GENERAL_6R_DEBUG("theta[6] is invalid, discarding! (s6 = %.5f, c6 = %.5f)!\n", s6, c6);
						continue;
					}

					j6 = base_angle(atan2(s6, c6));
					GENERAL_6R_DEBUG("theta[6] = %.5f\n", j6);

					/* See if it's within the joint limits */
					if (limits->test_joint_limits && !is_within_limit(robot, 6, j6))
					{
						GENERAL_6R_DEBUG("theta[6] is not within joint limits, skipping!\n");
						continue;
					}

					/* ======================================================================
					 * STEP 7: Eliminate duplicates and store solution
					 * ====================================================================*/

					for (m = 1; m <= *no_solutions; m++)
					{
						double diff = fabs(base_angle(fabs(J[_2D_IDX(1,m,6,16)] - j1[l]))) +
						              fabs(base_angle(fabs(J[_2D_IDX(2,m,6,16)] - j2[k]))) +
						              fabs(base_angle(fabs(J[_2D_IDX(3,m,6,16)] - s345[j].j3))) +
						              fabs(base_angle(fabs(J[_2D_IDX(4,m,6,16)] - s345[j].j4))) +
						              fabs(base_angle(fabs(J[_2D_IDX(5,m,6,16)] - s345[j].j5))) +
						              fabs(base_angle(fabs(J[_2D_IDX(6,m,6,16)] - j6)));

						if (diff < limits->dulpicate_solution_tol)
						{
							duplication = 1;
							break;
						}
					}

					if (!duplication)
					{
						(*no_solutions)++;

						if (flags)
						{
							if (sol_newton_optimized)
								flags[0] |= FLAG1_SOL_N_ALTERNATE_METHOD(*no_solutions);

							if (sol_alternate_method)
								flags[0] |= FLAG1_SOL_N_OPTIMIZED_ON_Z(*no_solutions);

							if (sol_singular)
								flags[1] |= FLAG2_SOL_N_SINGULAR(*no_solutions);
						}

						J[_2D_IDX(1,*no_solutions,6,16)] = j1[l];
						J[_2D_IDX(2,*no_solutions,6,16)] = j2[k];
						J[_2D_IDX(3,*no_solutions,6,16)] = s345[j].j3;
						J[_2D_IDX(4,*no_solutions,6,16)] = s345[j].j4;
						J[_2D_IDX(5,*no_solutions,6,16)] = s345[j].j5;
						J[_2D_IDX(6,*no_solutions,6,16)] = j6;
						GENERAL_6R_DEBUG("Solution stored!\n");
					}
					else
						GENERAL_6R_DEBUG("The solution is a duplicate, discarded!\n");
				}
			}
		}
	}

	/* Finally add offsets */
	for (m = 1; m <= *no_solutions; m++)
	{
		J[_2D_IDX(1,m,6,16)] = base_angle(J[_2D_IDX(1,m,6,16)] - robot->denavit_hartenberg[DH_THETA(1)]);
		J[_2D_IDX(2,m,6,16)] = base_angle(J[_2D_IDX(2,m,6,16)] - robot->denavit_hartenberg[DH_THETA(2)]);
		J[_2D_IDX(3,m,6,16)] = base_angle(J[_2D_IDX(3,m,6,16)] - robot->denavit_hartenberg[DH_THETA(3)]);
		J[_2D_IDX(4,m,6,16)] = base_angle(J[_2D_IDX(4,m,6,16)] - robot->denavit_hartenberg[DH_THETA(4)]);
		J[_2D_IDX(5,m,6,16)] = base_angle(J[_2D_IDX(5,m,6,16)] - robot->denavit_hartenberg[DH_THETA(5)]);
		J[_2D_IDX(6,m,6,16)] = base_angle(J[_2D_IDX(6,m,6,16)] - robot->denavit_hartenberg[DH_THETA(6)]);
	}
}
