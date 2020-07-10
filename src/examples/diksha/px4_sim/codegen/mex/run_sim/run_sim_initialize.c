/*
 * run_sim_initialize.c
 *
 * Code generation for function 'run_sim_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "run_sim_initialize.h"
#include "_coder_run_sim_mex.h"
#include "run_sim_data.h"

/* Function Declarations */
static void run_sim_once(void);

/* Function Definitions */
static void run_sim_once(void)
{
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m",
                  0, 1, 3, 0, 0, 0, 0, 1, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0, 0, "run_sim", 0, -1, 779);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0, 2, 679, -1, 774);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0, 1, 333, -1, 659);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0, 0, 38, -1, 307);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0, 0, 309, 329, 668);

  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\time_trajj.m",
                  1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 1, 0, "time_trajj", 0, -1, 373);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1, 0, 63, -1, 368);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 1U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\local.m", 2,
                  1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 2, 0, "local", 0, -1, 319);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 2, 0, 48, -1, 314);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 2U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\controller.m",
                  3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 3, 0, "controller", 0, -1, 1622);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 3, 0, 92, -1, 1599);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 3U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\eul2rotm.m",
                  4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 4, 0, "eul2rotm", 0, -1, 508);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 4, 0, 39, -1, 503);

  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 4U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\AeroFEst.m",
                  5, 1, 6, 2, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 5, 0, "AeroFEst", 0, -1, 1997);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 5, 1071, -1, 1991);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 4, 931, -1, 950);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 3, 850, -1, 920);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 2, 688, -1, 752);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 1, 675, -1, 681);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5, 0, 142, -1, 662);

  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 5, 0, 665, 674, 828, 955);
  covrtIfInit(&emlrtCoverageInstance, 5, 1, 828, 845, 922, 955);

  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 5U);

  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\AeroMEst.m",
                  6, 1, 6, 2, 0, 0, 0, 0, 0, 0, 0);

  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 6, 0, "AeroMEst", 0, -1, 1469);

  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 5, 769, -1, 1463);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 4, 743, -1, 762);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 3, 662, -1, 732);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 2, 573, -1, 637);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 1, 560, -1, 566);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6, 0, 141, -1, 547);

  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 6, 0, 550, 559, 640, 767);
  covrtIfInit(&emlrtCoverageInstance, 6, 1, 640, 657, 734, 767);

  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 6U);
}

void run_sim_initialize(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    run_sim_once();
  }
}

/* End of code generation (run_sim_initialize.c) */
