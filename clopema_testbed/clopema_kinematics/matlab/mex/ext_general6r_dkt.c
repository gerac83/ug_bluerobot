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

/* Create denavit hartenberg matrix */
void denavit_hartenberg_matrix(double* M, gen6r_robot_t* robot, int joint, double theta)
{
	double s, c, u, l, a, d;

	/* add theta offset */
	theta += robot->denavit_hartenberg[DH_THETA(joint)];

	s = sin(theta);
	c = cos(theta);
	u = sin(robot->denavit_hartenberg[DH_ALPHA(joint)]);
	l = cos(robot->denavit_hartenberg[DH_ALPHA(joint)]);
	a = robot->denavit_hartenberg[DH_A(joint)];
	d = robot->denavit_hartenberg[DH_D(joint)];

	GENERAL_6R_DEBUG("Joint %d, theta (with offset) %.5f\n", joint, theta);

	memset(M, 0, sizeof(double) * (4*4));

	/* column 1 */
	M[_2D_IDX(1,1,4,4)] = c;
	M[_2D_IDX(2,1,4,4)] = s;

	/* column 2 */
	M[_2D_IDX(1,2,4,4)] = -s*l;
	M[_2D_IDX(2,2,4,4)] = c*l;
	M[_2D_IDX(3,2,4,4)] = u;

	/* column 3 */
	M[_2D_IDX(1,3,4,4)] = s*u;
	M[_2D_IDX(2,3,4,4)] = -c*u;
	M[_2D_IDX(3,3,4,4)] = l;

	/* column 4 */
	M[_2D_IDX(1,4,4,4)] = a*c;
	M[_2D_IDX(2,4,4,4)] = a*s;
	M[_2D_IDX(3,4,4,4)] = d;
	M[_2D_IDX(4,4,4,4)] = 1;
}

void general_6r_dkt(double* MhV, gen6r_robot_t* robot, double* J)
{
	double A[4*4], B[4*4], M[4*4];
	double *R, *P;
	int i;

	/* Generate eye matrix */
	memset(A, 0, sizeof(A));
	A[_2D_IDX(1,1,4,4)] = 1;
	A[_2D_IDX(2,2,4,4)] = 1;
	A[_2D_IDX(3,3,4,4)] = 1;
	A[_2D_IDX(4,4,4,4)] = 1;

	R = B;
	P = A;

	/* Do the matrix multiplication */
	for (i = 0; i < 6; i++)
	{
		denavit_hartenberg_matrix(M, robot, i + 1, J[i]);
		matrix_mult(R, P, 4, 4, M, 4, 4);

		if (R == B)
		{
			R = A;
			P = B;
		}
		else
		{
			R = B;
			P = A;
		}
	}

	/* Copy the result */
	copy_matrix(MhV, P, 4, 4);
}
