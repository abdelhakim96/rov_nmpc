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


#ifndef NMPC_COMMON_H
#define NMPC_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup NMPC ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define NMPC_QPOASES  0
#define NMPC_QPOASES3 1
/** FORCES QP solver indicator.*/
#define NMPC_FORCES   2
/** qpDUNES QP solver indicator.*/
#define NMPC_QPDUNES  3
/** HPMPC QP solver indicator. */
#define NMPC_HPMPC    4
#define NMPC_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define NMPC_QP_SOLVER NMPC_QPOASES

#include "nmpc_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define NMPC_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define NMPC_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define NMPC_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define NMPC_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define NMPC_N 30
/** Number of online data values. */
#define NMPC_NOD 3
/** Number of path constraints. */
#define NMPC_NPAC 0
/** Number of control variables. */
#define NMPC_NU 4
/** Number of differential variables. */
#define NMPC_NX 9
/** Number of algebraic variables. */
#define NMPC_NXA 0
/** Number of differential derivative variables. */
#define NMPC_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define NMPC_NY 13
/** Number of references/measurements on the last (N + 1)st node. */
#define NMPC_NYN 9
/** Total number of QP optimization variables. */
#define NMPC_QP_NV 129
/** Number of integration steps per shooting interval. */
#define NMPC_RK_NIS 1
/** Number of Runge-Kutta stages per integration step. */
#define NMPC_RK_NSTAGES 4
/** Providing interface for arrival cost. */
#define NMPC_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define NMPC_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define NMPC_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct NMPCvariables_
{
int dummy;
/** Matrix of size: 31 x 9 (row major format)
 * 
 *  Matrix containing 31 differential variable vectors.
 */
real_t x[ 279 ];

/** Matrix of size: 30 x 4 (row major format)
 * 
 *  Matrix containing 30 control variable vectors.
 */
real_t u[ 120 ];

/** Matrix of size: 31 x 3 (row major format)
 * 
 *  Matrix containing 31 online data vectors.
 */
real_t od[ 93 ];

/** Column vector of size: 390
 * 
 *  Matrix containing 30 reference/measurement vectors of size 13 for first 30 nodes.
 */
real_t y[ 390 ];

/** Column vector of size: 9
 * 
 *  Reference/measurement vector for the 31. node.
 */
real_t yN[ 9 ];

/** Matrix of size: 13 x 13 (row major format) */
real_t W[ 169 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t WN[ 81 ];

/** Column vector of size: 9
 * 
 *  Current state feedback vector.
 */
real_t x0[ 9 ];


} NMPCvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct NMPCworkspace_
{
/** Column vector of size: 664 */
real_t rhs_aux[ 664 ];

real_t rk_ttt;

/** Row vector of size: 133 */
real_t rk_xxx[ 133 ];

/** Matrix of size: 4 x 126 (row major format) */
real_t rk_kkk[ 504 ];

/** Row vector of size: 133 */
real_t state[ 133 ];

/** Column vector of size: 270 */
real_t d[ 270 ];

/** Column vector of size: 390 */
real_t Dy[ 390 ];

/** Column vector of size: 9 */
real_t DyN[ 9 ];

/** Matrix of size: 270 x 9 (row major format) */
real_t evGx[ 2430 ];

/** Matrix of size: 270 x 4 (row major format) */
real_t evGu[ 1080 ];

/** Row vector of size: 16 */
real_t objValueIn[ 16 ];

/** Row vector of size: 13 */
real_t objValueOut[ 13 ];

/** Matrix of size: 270 x 9 (row major format) */
real_t Q1[ 2430 ];

/** Matrix of size: 270 x 13 (row major format) */
real_t Q2[ 3510 ];

/** Matrix of size: 120 x 4 (row major format) */
real_t R1[ 480 ];

/** Matrix of size: 120 x 13 (row major format) */
real_t R2[ 1560 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t QN1[ 81 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t QN2[ 81 ];

/** Column vector of size: 9 */
real_t Dx0[ 9 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t T[ 81 ];

/** Matrix of size: 4185 x 4 (row major format) */
real_t E[ 16740 ];

/** Matrix of size: 4185 x 4 (row major format) */
real_t QE[ 16740 ];

/** Matrix of size: 270 x 9 (row major format) */
real_t QGx[ 2430 ];

/** Column vector of size: 270 */
real_t Qd[ 270 ];

/** Column vector of size: 279 */
real_t QDy[ 279 ];

/** Matrix of size: 120 x 9 (row major format) */
real_t H10[ 1080 ];

/** Matrix of size: 129 x 129 (row major format) */
real_t H[ 16641 ];

/** Column vector of size: 129 */
real_t g[ 129 ];

/** Column vector of size: 129 */
real_t lb[ 129 ];

/** Column vector of size: 129 */
real_t ub[ 129 ];

/** Column vector of size: 129 */
real_t x[ 129 ];

/** Column vector of size: 129 */
real_t y[ 129 ];


} NMPCworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int nmpc_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void nmpc_rhs_forw(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int nmpc_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int nmpc_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int nmpc_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void nmpc_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 31 with xEnd. 2. Initialize node 31 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmpc_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t nmpc_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t nmpc_getObjective(  );


/* 
 * Extern declarations. 
 */

extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* NMPC_COMMON_H */
