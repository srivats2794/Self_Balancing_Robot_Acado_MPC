/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES3

#include "acado_qpoases3_interface.h"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 100
/** Number of online data values. */
#define ACADO_NOD 0
/** Number of path constraints. */
#define ACADO_NPAC 2
/** Number of control variables. */
#define ACADO_NU 2
/** Number of differential variables. */
#define ACADO_NX 7
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 7
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 9
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 7
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 200
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 2
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 1
/** Single versus double precision data type representation. */
#define ACADO_SINGLE_PRECISION 0
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Column vector of size: 1400 */
real_t rk_Ktraj[ 1400 ];

/** Column vector of size: 12600 */
real_t rk_diffKtraj[ 12600 ];

/** Matrix of size: 100 x 7 (row major format) */
real_t rk_Xprev[ 700 ];

/** Matrix of size: 100 x 2 (row major format) */
real_t rk_Uprev[ 200 ];

/** Matrix of size: 101 x 7 (row major format)
 * 
 *  Matrix containing 101 differential variable vectors.
 */
real_t x[ 707 ];

/** Matrix of size: 100 x 2 (row major format)
 * 
 *  Matrix containing 100 control variable vectors.
 */
real_t u[ 200 ];

/** Column vector of size: 900
 * 
 *  Matrix containing 100 reference/measurement vectors of size 9 for first 100 nodes.
 */
real_t y[ 900 ];

/** Column vector of size: 7
 * 
 *  Reference/measurement vector for the 101. node.
 */
real_t yN[ 7 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t W[ 81 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t WN[ 49 ];

/** Column vector of size: 7
 * 
 *  Current state feedback vector.
 */
real_t x0[ 7 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim7_swap;

/** Matrix of size: 7 x 10 (row major format) */
real_t rk_dim7_bPerm[ 70 ];

/** Column vector of size: 7 */
real_t rhs_aux[ 7 ];

real_t rk_ttt;

/** Row vector of size: 16 */
real_t rk_xxx[ 16 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t rk_A[ 49 ];

/** Matrix of size: 7 x 10 (row major format) */
real_t rk_b[ 70 ];

/** Row vector of size: 7 */
int rk_dim7_perm[ 7 ];

/** Column vector of size: 14 */
real_t rk_rhsTemp[ 14 ];

/** Row vector of size: 112 */
real_t rk_diffsTemp2[ 112 ];

/** Matrix of size: 7 x 9 (row major format) */
real_t rk_diffsPrev2[ 63 ];

/** Matrix of size: 7 x 9 (row major format) */
real_t rk_diffsNew2[ 63 ];

/** Row vector of size: 7 */
real_t rk_stageValues[ 7 ];

/** Row vector of size: 9 */
real_t rk_delta[ 9 ];

/** Row vector of size: 7 */
real_t rk_xxx_lin[ 7 ];

/** Matrix of size: 2 x 7 (row major format) */
real_t rk_Khat[ 14 ];

/** Column vector of size: 14 */
real_t rk_Xhat[ 14 ];

/** Column vector of size: 63 */
real_t rk_diffKtraj_aux[ 63 ];

/** Row vector of size: 72 */
real_t state[ 72 ];

/** Column vector of size: 700 */
real_t d[ 700 ];

/** Column vector of size: 900 */
real_t Dy[ 900 ];

/** Column vector of size: 7 */
real_t DyN[ 7 ];

/** Matrix of size: 700 x 7 (row major format) */
real_t evGx[ 4900 ];

/** Matrix of size: 700 x 2 (row major format) */
real_t evGu[ 1400 ];

/** Row vector of size: 9 */
real_t objValueIn[ 9 ];

/** Row vector of size: 9 */
real_t objValueOut[ 9 ];

/** Matrix of size: 700 x 7 (row major format) */
real_t Q1[ 4900 ];

/** Matrix of size: 700 x 9 (row major format) */
real_t Q2[ 6300 ];

/** Matrix of size: 200 x 2 (row major format) */
real_t R1[ 400 ];

/** Matrix of size: 200 x 9 (row major format) */
real_t R2[ 1800 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t QN1[ 49 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t QN2[ 49 ];

/** Column vector of size: 38 */
real_t conAuxVar[ 38 ];

/** Row vector of size: 9 */
real_t conValueIn[ 9 ];

/** Row vector of size: 20 */
real_t conValueOut[ 20 ];

/** Column vector of size: 200 */
real_t evH[ 200 ];

/** Matrix of size: 200 x 7 (row major format) */
real_t evHx[ 1400 ];

/** Matrix of size: 200 x 2 (row major format) */
real_t evHu[ 400 ];

/** Column vector of size: 2 */
real_t evHxd[ 2 ];

/** Column vector of size: 707 */
real_t sbar[ 707 ];

/** Column vector of size: 7 */
real_t Dx0[ 7 ];

/** Matrix of size: 700 x 7 (row major format) */
real_t C[ 4900 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t W1[ 14 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t W2[ 14 ];

/** Matrix of size: 35350 x 2 (row major format) */
real_t E[ 70700 ];

/** Column vector of size: 707 */
real_t QDy[ 707 ];

/** Column vector of size: 7 */
real_t w1[ 7 ];

/** Column vector of size: 7 */
real_t w2[ 7 ];

/** Matrix of size: 200 x 200 (row major format) */
real_t H[ 40000 ];

/** Matrix of size: 800 x 200 (row major format) */
real_t A[ 160000 ];

/** Column vector of size: 200 */
real_t g[ 200 ];

/** Column vector of size: 200 */
real_t lb[ 200 ];

/** Column vector of size: 200 */
real_t ub[ 200 ];

/** Column vector of size: 800 */
real_t lbA[ 800 ];

/** Column vector of size: 800 */
real_t ubA[ 800 ];

/** Column vector of size: 200 */
real_t x[ 200 ];

/** Column vector of size: 1000 */
real_t y[ 1000 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_index Number of the shooting interval.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int rk_index );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_acado_diffs(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 101 with xEnd. 2. Initialize node 101 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_getObjective(  );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
