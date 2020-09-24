#include "acado.h"

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

/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/
/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
using namespace std;
/* A template for testing of the solver. */
void init_acado()
{
	/* Initialize the solver. */
	acado_initializeSolver();
	for (int i = 0; i < NX * (N + 1); ++i)
		acadoVariables.x[i] = 0.0;
	for (int i = 0; i < NU * N; ++i)
		acadoVariables.u[i] = 0.0;

	// /* Initialize the measurements/reference. */
	// for (i = 0; i < NY * N; ++i)
	// 	acadoVariables.y[i] = 0.0;
	// for (i = 0; i < NYN; ++i)
	// 	acadoVariables.yN[i] = 0.0;
}
void run_mpc_acado(std::vector<double> states, std::vector<double> path)
{
	/* Some temporary variables. */
	int i, iter;
	acado_timer t;

	//previou
	acado_shiftStates(1, 0, 0);
	//
	acado_shiftControls(0);

	for (i = 0; i < NX; ++i)
	{
		acadoVariables.x[i] = states[i];
		acadoVariables.x0[i] = states[i];
	}

	for (i = 0; i < NU; ++i)
		acadoVariables.u[i] = 0; // previous u

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)
		acadoVariables.y[i] = 0.0;
	for (i = 0; i < NYN; ++i)
		acadoVariables.yN[i] = 0.0;

	/* MPC: initialize the current state feedback. */
	// #if ACADO_INITIAL_STATE_FIXED
	// 	for (i = 0; i < NX; ++i)
	// 		acadoVariables.x0[i] = 0.1;
	// #endif

	if (VERBOSE)
		acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic(&t);

	/* The "real-time iterations" loop. */

	/* Perform the feedback step. */
	acado_feedbackStep();

	/* Apply the new control immediately to the process, first NU components. */

	if (VERBOSE)
		printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT());

	/* Optional: shift the initialization (look at acado_common.h). */
	/* acado_shiftStates(2, 0, 0); */
	/* acado_shiftControls( 0 ); */

	/* Read the elapsed time. */
	real_t te = acado_toc(&t);

	if (VERBOSE)
		printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if (!VERBOSE)
		printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();
}