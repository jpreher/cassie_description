/*
 * MIT License
 * 
 * Copyright (c) 2020 Jenna Reher (jreher@caltech.edu)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#define S_FUNCTION_NAME  evaluate_holonomics
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include "J_achilles_constraint.hpp"
#include "Jdot_achilles_constraint.hpp"
#include "J_left_fixed_constraint.hpp"
#include "Jdot_left_fixed_constraint.hpp"
#include "J_right_fixed_constraint.hpp"
#include "Jdot_right_fixed_constraint.hpp"
#include "J_leftSole_constraint.hpp"
#include "Jdot_leftSole_constraint.hpp"
#include "J_rightSole_constraint.hpp"
#include "Jdot_rightSole_constraint.hpp"

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 2))
        return;
    
    ssSetInputPortWidth(S, 0, 22); // q
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    ssSetInputPortWidth(S, 1, 22); // dq
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 1, 1);
    
    if (!ssSetNumOutputPorts(S, 10))
        return;

    ssSetOutputPortMatrixDimensions(S, 0, 2, 22);  // J_ach
    ssSetOutputPortMatrixDimensions(S, 1, 2, 22);  // J_left_fixed
    ssSetOutputPortMatrixDimensions(S, 2, 2, 22);  // J_right_fixed
    ssSetOutputPortMatrixDimensions(S, 3, 5, 22);  // J_left_sole
    ssSetOutputPortMatrixDimensions(S, 4, 5, 22);  // J_right_sole
    
    ssSetOutputPortMatrixDimensions(S, 5, 2, 22);  // dJ_ach
    ssSetOutputPortMatrixDimensions(S, 6, 2, 22);  // dJ_left_fixed
    ssSetOutputPortMatrixDimensions(S, 7, 2, 22);  // dJ_right_fixed
    ssSetOutputPortMatrixDimensions(S, 8, 5, 22);  // dJ_left_sole
    ssSetOutputPortMatrixDimensions(S, 9, 5, 22);  // dJ_right_sole

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Do stuff.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Pull inputs
    const real_T* q  = ssGetInputPortRealSignal(S,0);
    const real_T* dq = ssGetInputPortRealSignal(S,1);

    // Create output variables
    real_T * J_ach         = ssGetOutputPortRealSignal(S,0);
    real_T * J_left_fixed  = ssGetOutputPortRealSignal(S,1);
    real_T * J_right_fixed = ssGetOutputPortRealSignal(S,2);
    real_T * J_left_sole   = ssGetOutputPortRealSignal(S,3);
    real_T * J_right_sole  = ssGetOutputPortRealSignal(S,4);
    real_T * dJ_ach         = ssGetOutputPortRealSignal(S,5);
    real_T * dJ_left_fixed  = ssGetOutputPortRealSignal(S,6);
    real_T * dJ_right_fixed = ssGetOutputPortRealSignal(S,7);
    real_T * dJ_left_sole   = ssGetOutputPortRealSignal(S,8);
    real_T * dJ_right_sole  = ssGetOutputPortRealSignal(S,9);

    // Evaluate the raw source functions
	SymFunction::J_achilles_constraint_raw((double *)J_ach, (const double *)q);
    SymFunction::J_left_fixed_constraint_raw((double *)J_left_fixed, (const double *)q);
    SymFunction::J_right_fixed_constraint_raw((double *)J_right_fixed, (const double *)q);
    SymFunction::J_leftSole_constraint_raw((double *)J_left_sole, (const double *)q);
    SymFunction::J_rightSole_constraint_raw((double *)J_right_sole, (const double *)q);
    
    SymFunction::Jdot_achilles_constraint_raw((double *)dJ_ach, (const double *)q, (const double *)dq);
    SymFunction::Jdot_left_fixed_constraint_raw((double *)dJ_left_fixed, (const double *)q, (const double *)dq);
    SymFunction::Jdot_right_fixed_constraint_raw((double *)dJ_right_fixed, (const double *)q, (const double *)dq);
    SymFunction::Jdot_leftSole_constraint_raw((double *)dJ_left_sole, (const double *)q, (const double *)dq);
    SymFunction::Jdot_rightSole_constraint_raw((double *)dJ_right_sole, (const double *)q, (const double *)dq);
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
