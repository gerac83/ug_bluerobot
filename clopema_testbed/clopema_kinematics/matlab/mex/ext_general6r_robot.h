/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*    Authors: Meloun Martin <meloumar@cmp.felk.cvut.cz>
*             Tomas Pajdla  <pajdla@cmp.felk.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#ifndef EXT_GENERAL6R_ROBOT_H
#define EXT_GENERAL6R_ROBOT_H

/* Robot definition structure */

#define DH_ALPHA(joint)               ((joint)      - 1)
#define DH_A(joint)                   ((joint) + 6  - 1)
#define DH_THETA(joint)               ((joint) + 12 - 1)
#define DH_D(joint)                   ((joint) + 18 - 1)

#define ROB_DH_ALPHA(robot,joint)     robot->denavit_hartenberg[DH_ALPHA(joint)]
#define ROB_DH_A(robot,joint)         robot->denavit_hartenberg[DH_A(joint)]
#define ROB_DH_THETA(robot,joint)     robot->denavit_hartenberg[DH_THETA(joint)]
#define ROB_DH_D(robot,joint)         robot->denavit_hartenberg[DH_D(joint)]

#define JOINT_MIN(joint)              ((joint)     - 1)
#define JOINT_MAX(joint)              ((joint) + 6 - 1)

#define ROB_JOINT_MIN(robot,joint)    robot->joint_limits[JOINT_MIN(joint)]
#define ROB_JOINT_MAX(robot,joint)    robot->joint_limits[JOINT_MAX(joint)]

struct __gen6r_robot
{
	/* Robot name */
	const char* name;

	/* DH notation { alpha, a, theta, d } */
	double denavit_hartenberg[6 * 4];

	/* Joint limits */
	double joint_limits[6 * 2];

	/* Function pointers to obtain pregenerated matrices */

	/* Pa, Pb, Pc are 14x9 matrices, N is 14x8 matrix: these are populated based on supplied MhV */
	void (*form_base_ikt_matrices)(struct __gen6r_robot* robot, double* N, double* Pa, double* Pb, double* Pc, double* MhV);

	/* M6l is 6x2 matrix, M6r is 6x1 matrix, which are populated based on the rest of the parameters */
	void (*form_j6_ikt_matrices)(struct __gen6r_robot* robot, double* M6l, double* M6r, double* MhV, double s1, double c1,
															 double s2, double c2, double s3, double c3, double s4, double c4, double s5, double c5);

	/* For testing purposes, create a singular position (returns MhV matrix) */
	void (*create_singular_position)(struct __gen6r_robot* robot, double* MhV);

};

typedef struct __gen6r_robot gen6r_robot_t;

#endif //!EXT_GENERAL6R_ROBOT_H
