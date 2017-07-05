/**
* Copyright (c) CTU in Prague  - All Rights Reserved
* Created on: Oct 15, 2013
*    Authors: Meloun Martin <meloumar@cmp.felk.cvut.cz>
*             Tomas Pajdla  <pajdla@cmp.felk.cvut.cz>
*  Institute: Czech Technical University in Prague
*/

#ifndef EXT_GENERAL6R_OPWRAP_H
#define EXT_GENERAL6R_OPWRAP_H

#include "math.h"

#include "assert.h"


#ifndef mex_h
#define ptrdiff_t int
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "blas.h"
#include "lapack.h"
#ifdef __cplusplus
}
#endif

#ifndef dgemm
#define dgemm dgemm_
#endif

#ifndef dggev
#define dggev dggev_
#endif

#ifndef dgesvd
#define dgesvd dgesvd_
#endif

#ifndef dgeqrf
#define dgeqrf dgeqrf_
#endif

#ifndef dgetrf
#define dgetrf dgetrf_
#endif


#if defined(_MSC_VER)
# define inline __inline
#elif defined(__GNUC__)
# define inline __attribute__((__inline__))
#else
# define inline
#endif

/* Constants */
#define M_EPS     2.2e-16
#define M_BIGEPS  2.2e-10

#define M_PI      3.14159265358979323846
#define M_PI_2    1.57079632679489661923
#define M_3_PI_2  4.71238898038468985769
#define DEG(m)    (M_PI/180.0 * (m))

#define SGN(x)    (((x) > 0) - ((x) < 0))

/* Array size macro */
#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof( (x)[0]) )
#endif

#ifndef MIN
# define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
# define MAX(a,b) (((a)>(b))?(a):(b))
#endif

/* Matrix indexing (FORTRAN) */
#define _2D_IDX(x,y,xl,yl)    (((y)-1)*(xl)+((x)-1))
#define _2D_IDX_T(x,y,xl,yl)  (((x)-1)*(yl)+((y)-1))

/* If debugging is disabled then define it as empty macro */
#ifndef GENERAL_6R_DEBUG
# define GENERAL_6R_DEBUG(...) do {} while(0)
#else
# define GENERAL_6R_DEBUGGING
#endif

/* Base angle */
inline double base_angle(double m)
{
	m = fmod(m, 2*M_PI);

	if (m > M_PI)
		m -= 2*M_PI;
	else if (m <= -M_PI)
		m += 2*M_PI;

	return m;
}

/* Matrix transposition */
inline void transpose_matrix(double* At, double* A, int m, int n)
{
	size_t i, j;

	for (i = 1; i <= m; i++)
		for (j = 1; j <= n; j++)
			At[_2D_IDX_T(i,j,m,n)] = A[_2D_IDX(i,j,m,n)];
}

/* Copy matrix */
inline void copy_matrix(double* An, double* A, int m, int n)
{
	memcpy(An, A, m*n*sizeof(double));
}

/* Matrix multiplication wrappers */
inline void matrix_mult_ld(double* C, double* A, char* ta, ptrdiff_t lda, ptrdiff_t ma, ptrdiff_t na, double* B, char* tb, ptrdiff_t ldb, ptrdiff_t mb, ptrdiff_t nb)
{
	/* BLAS constants */
	double ONE = 1.0, ZERO = 0.0;

	/* Check if the sizes match */
	assert(na == mb);

	/* Call BLAS routine */
	dgemm(ta, tb, &ma, &nb, &mb, &ONE, A, &lda, B, &ldb, &ZERO, C, &ma);
}

inline void matrix_mult_ex(double* C, double* A, char* ta, ptrdiff_t ma, ptrdiff_t na, double* B, char* tb, ptrdiff_t mb, ptrdiff_t nb)
{
	matrix_mult_ld(C, A, "N", ma, ma, na, B, "N", mb, mb, nb);
}

inline void matrix_mult(double* C, double* A, ptrdiff_t ma, ptrdiff_t na, double* B, ptrdiff_t mb, ptrdiff_t nb)
{
	matrix_mult_ex(C, A, "N", ma, na, B, "N", mb, nb);
}

/* Singular value decomposition */
inline int svd(double* U, double* S, double* Vt, double* A, ptrdiff_t m, ptrdiff_t n)
{
	char jobu[2], jobvt[2];
	ptrdiff_t lwork = -1;
	ptrdiff_t info;

	/* Work */
	double work_size;

	/* NULL arguments on U or Vt specify no job done for them */
	if (!U)
		jobu[0] = 'N';
	else
		jobu[0] = 'A';

	jobu[1] = '\0';

	if (!Vt)
		jobvt[0] = 'N';
	else
		jobvt[0] = 'A';

	jobvt[1] = '\0';

	dgesvd(jobu, jobvt, &m, &n, A, &m, S, U, &m, Vt, &n, &work_size, &lwork, &info);
	if (info)
		return 1;

	lwork = (ptrdiff_t) work_size;
	{
#if __STDC_VERSION__ >= 199901L
		double work[lwork];
#else
		/* Create work space */
		double* work = (double*)malloc(lwork * sizeof(double));
#endif

		dgesvd(jobu, jobvt, &m, &n, A, &m, S, U, &m, Vt, &n, work, &lwork, &info);
		if (info)
			return 1;

#if __STDC_VERSION__ < 199901L
		/* Cleanup workspace */
		free(work);
#endif
	}
	return 0;
}

/* QR decomposition */
inline int qr(double* Q, double* R, double* A, ptrdiff_t m, ptrdiff_t n)
{
	/* Constants */
	double ONE = 1.0, ZERO = 0.0;

	/* Variables */
	int i, j, l;
	ptrdiff_t k = MIN(m,n);
	ptrdiff_t info;
	ptrdiff_t lwork = -1;
	ptrdiff_t single = 1;

	/* Pointers to working matrices */
	double *P, *S;

	/* Work */
	double work_size;

#if __STDC_VERSION__ >= 199901L
	double H[m*m];
	double W[m*m];
	double tau[k];
#else
	double* H =(double*) malloc(m * m * sizeof(double));
	double* W =(double*) malloc(m * m * sizeof(double));
	double* tau =(double*) malloc(k * sizeof(double));
#endif

	dgeqrf(&m, &n, A, &m, tau, &work_size, &lwork, &info);
	if (info)
		return 1;

	lwork = (ptrdiff_t) work_size;
	{

#if __STDC_VERSION__ >= 199901L
		double work[lwork];
#else
		/* Create work space */
		double* work =(double*) malloc(lwork * sizeof(double));
#endif

		dgeqrf(&m, &n, A, &m, tau, work, &lwork, &info);
		if (info)
			return 1;

		/* Copy R */
		for (i = 1; i <= m; i++)
			for (j = i; j <= n; j++)
			{
				R[_2D_IDX(i,j,m,n)] = A[_2D_IDX(i,j,m,n)];

				if (i == j)
					A[_2D_IDX(i,j,m,n)] = 1.0;
				else
					A[_2D_IDX(i,j,m,n)] = 0.0;
			}

		/* Calculate Q:
		 *
		 * Columns of A now are vectors vi, and you get
		 * Hi = I - tau * vi * vi^T
		 * and then
		 * Q = H1 * H2 * ... * Hi.
		 */

		/* Init Q and W */
		for (i = 1; i <= m; i++)
			Q[_2D_IDX(i,i,m,m)] = 1;

		memset(W, 0, sizeof(W));

		S = W;
		P = Q;

		/* Calculate Q = H1 * H2 * ... * Hi */
		for (l = 1; l <= k; l++)
		{
			/* Calculate Hi */
			dgemm("N", "T", &m, &m, &single, &ONE, &(A[_2D_IDX(1,l,m,n)]), &m, &(A[_2D_IDX(1,l,m,n)]), &m, &ZERO, H, &m);

			for (i = 1; i <= m; i++)
				for (j = 1; j <= m; j++)
				{
					if (i == j)
						H[_2D_IDX(i,j,m,m)] = 1.0 - tau[l-1]*H[_2D_IDX(i,j,m,m)];
					else
						H[_2D_IDX(i,j,m,m)] = -tau[l-1]*H[_2D_IDX(i,j,m,m)];
				}

			/* Multiply it to accumulate Q */
			matrix_mult(S, P, m, m, H, m, m);

			if (S == W)
			{
				S = Q;
				P = W;
			}
			else
			{
				S = W;
				P = Q;
			}
		}

		/* Check if the last multiplying was stored to W, in that case
		 * we have to copy it to Q
		 */
		if (S == Q)
			copy_matrix(Q, W, (int)m, (int)m);

#if __STDC_VERSION__ < 199901L
		/* Cleanup workspace */
		free(work);
#endif
	}

#if __STDC_VERSION__ < 199901L
	/* Cleanup matrices */
	free(tau);
	free(W);
	free(H);
#endif
	return 0;
}

/* Generalized eigenvalue problem for A*x = lambda*B*x */
inline int geneig(double* A, double* B, ptrdiff_t n, double* alpha_r, double* alpha_i, double* beta, double* V)
{
	/* Dummy constants */
	double DUMMY = 0;
	ptrdiff_t DUMMY_N = 1;

	/* Variables */
	ptrdiff_t info;
	ptrdiff_t lwork = -1;

	/* Work */
	double work_size;

	if (V)
		dggev("N", "V", &n, A, &n, B, &n, alpha_r, alpha_i, beta, &DUMMY, &DUMMY_N, V, &n, &work_size, &lwork, &info);
	else
		dggev("N", "N", &n, A, &n, B, &n, alpha_r, alpha_i, beta, &DUMMY, &DUMMY_N, &DUMMY, &DUMMY_N, &work_size, &lwork, &info);

	if (info)
		return 1;

	lwork = (ptrdiff_t) work_size;
	{
#if __STDC_VERSION__ >= 199901L
		double work[lwork];
#else
		/* Create work space */
		double* work =(double*) malloc(lwork * sizeof(double));
#endif

		if (V)
			dggev("N", "V", &n, A, &n, B, &n, alpha_r, alpha_i, beta, &DUMMY, &DUMMY_N, V, &n, work, &lwork, &info);
		else
			dggev("N", "N", &n, A, &n, B, &n, alpha_r, alpha_i, beta, &DUMMY, &DUMMY_N, &DUMMY, &DUMMY_N, work, &lwork, &info);

		if (info)
			return (int)info;

#if __STDC_VERSION__ < 199901L
		/* Cleanup workspace */
		free(work);
#endif
	}
	return 0;
}

/* Quick LU decomposition */
inline int lu_quick(double* M, int* P, ptrdiff_t m, ptrdiff_t n)
{
	/* Quick version */
	ptrdiff_t info, p;
	int s;
#if __STDC_VERSION__ >= 199901L
	ptrdiff_t piv[MIN(m,n)];
#else
	ptrdiff_t* piv =(ptrdiff_t*) malloc(MIN(m,n) * sizeof(ptrdiff_t));
#endif

	/* Decompose it */
	dgetrf(&m, &n, M, &m, piv, &info);
	if(info < 0)
		return 1;

	/* Permutation */
	for (p = 0; p < m; p++)
		P[p] = (int) p;

	for (p = 0; p < MIN(m,n); p++)
	{
		/* Swap p and piv[p] - 1 rows */
		s = P[p];
		P[p] = P[piv[p] - 1];
		P[piv[p] - 1] = s;
	}

#if __STDC_VERSION__ < 199901L
	/* Cleanup workspace */
	free(piv);
#endif
	return 0;
}

/* LU decomposition */
inline int lu(double* L, double* U, int* P, double* M, ptrdiff_t m, ptrdiff_t n)
{
	/* If m = n, then L and U are both same dimension. Otherwise for m > n L is m x n
	 * and U is n x n or when m < n L is m x m and U is m x n.
	 *
	 * P is sized m x 1.
	 */

	ptrdiff_t info, p;
	int i, j, s;
#if __STDC_VERSION__ >= 199901L
	ptrdiff_t piv[MIN(m,n)];
#else
	ptrdiff_t* piv = (ptrdiff_t*)malloc(MIN(m,n) * sizeof(ptrdiff_t));
#endif

	/* Decompose it */
	dgetrf(&m, &n, M, &m, piv, &info);
	if(info < 0)
		return 1;

	memset(L, 0, m*MIN(m,n)*sizeof(double));
	memset(U, 0, MIN(m,n)*n*sizeof(double));

	/* Get L & U matrices from the result */
	for (i = 1; i <= MIN(m,n); i++)
		for (j = i; j <= n; j++)
			U[_2D_IDX(i,j,MIN(m,n),n)] = M[_2D_IDX(i,j,m,n)];

	for (i = 2; i <= m; i++)
		for (j = 1; j <= MIN(m,n) && j < i; j++)
			L[_2D_IDX(i,j,m,MIN(m,n))] = M[_2D_IDX(i,j,m,n)];

	/* Unit diagonal */
	for (i = 1; i <= MIN(m,n); i++)
		L[_2D_IDX(i,i,m,MIN(m,n))] = 1.0;

		/* Permutation */
	for (p = 0; p < m; p++)
		P[p] = (int) p;

	for (p = 0; p < MIN(m,n); p++)
	{
		/* Swap p and piv[p] - 1 rows */
		s = P[p];
		P[p] = P[piv[p] - 1];
		P[piv[p] - 1] = s;
	}

#if __STDC_VERSION__ < 199901L
		/* Cleanup workspace */
		free(piv);
#endif
	return 0;
}

#endif //!EXT_GENERAL6R_OPWRAP_H
