/*
 * run_sim.c
 *
 * Code generation for function 'run_sim'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "time_trajj.h"
#include "local.h"
#include "controller.h"
#include "run_sim_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 13, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo b_emlrtRSI = { 14, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo c_emlrtRSI = { 15, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo emlrtMCI = { 24, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo b_emlrtMCI = { 26, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo c_emlrtMCI = { 28, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo d_emlrtMCI = { 23, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo e_emlrtMCI = { 25, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtMCInfo f_emlrtMCI = { 27, 1, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo w_emlrtRSI = { 28, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo x_emlrtRSI = { 27, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo y_emlrtRSI = { 26, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo ab_emlrtRSI = { 24, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo bb_emlrtRSI = { 25, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

static emlrtRSInfo cb_emlrtRSI = { 23, "run_sim",
  "F:\\aerospace 2\\matlab ex\\animesh_mod\\px4_sim\\run_sim.m" };

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const real_T u[400]);
static const mxArray *c_emlrt_marshallOut(const char_T u);
static const mxArray *d_emlrt_marshallOut(const emlrtStack *sp, const char_T u[2]);
static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static const mxArray *e_emlrt_marshallOut(const emlrtStack *sp, const char_T u
  [20]);
static const mxArray *emlrt_marshallOut(const real_T u);
static void figure(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static void hold(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);
static void plot(const emlrtStack *sp, const mxArray *b, const mxArray *c, const
                 mxArray *d, emlrtMCInfo *location);
static void ylabel(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const real_T u[400])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv0[1] = { 400 };

  real_T *pData;
  int32_T i;
  y = NULL;
  m1 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  pData = (real_T *)mxGetPr(m1);
  for (i = 0; i < 400; i++) {
    pData[i] = u[i];
  }

  emlrtAssign(&y, m1);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const char_T u)
{
  const mxArray *y;
  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateString1(u);
  emlrtAssign(&y, m2);
  return y;
}

static const mxArray *d_emlrt_marshallOut(const emlrtStack *sp, const char_T u[2])
{
  const mxArray *y;
  const mxArray *m3;
  static const int32_T iv1[2] = { 1, 2 };

  y = NULL;
  m3 = emlrtCreateCharArray(2, iv1);
  emlrtInitCharArrayR2013a(sp, 2, m3, &u[0]);
  emlrtAssign(&y, m3);
  return y;
}

static void disp(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "disp", true, location);
}

static const mxArray *e_emlrt_marshallOut(const emlrtStack *sp, const char_T u
  [20])
{
  const mxArray *y;
  const mxArray *m4;
  static const int32_T iv2[2] = { 1, 20 };

  y = NULL;
  m4 = emlrtCreateCharArray(2, iv2);
  emlrtInitCharArrayR2013a(sp, 20, m4, &u[0]);
  emlrtAssign(&y, m4);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

static void figure(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "figure", true, location);
}

static void hold(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "hold", true, location);
}

static void plot(const emlrtStack *sp, const mxArray *b, const mxArray *c, const
                 mxArray *d, emlrtMCInfo *location)
{
  const mxArray *pArrays[3];
  pArrays[0] = b;
  pArrays[1] = c;
  pArrays[2] = d;
  emlrtCallMATLABR2012b(sp, 0, NULL, 3, pArrays, "plot", true, location);
}

static void ylabel(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "ylabel", true, location);
}

void run_sim(const emlrtStack *sp, real_T F_traj[400], real_T M_traj[1200])
{
  boolean_T b0;
  real_T time;
  real_T des_z[400];
  real_T curr_z[400];
  real_T ttraj[400];
  int32_T iter;
  real_T posd[3];
  real_T veld[3];
  real_T posc[3];
  real_T velc[3];
  static const char_T cv0[2] = { 'o', 'n' };

  real_T dv0[3];
  real_T dv1[3];
  real_T dv2[3];
  real_T dv3[3];
  real_T dv4[2];
  real_T M[3];
  static const char_T cv1[20] = { 'S', 'i', 'm', 'u', 'l', 'a', 't', 'i', 'o',
    'n', ' ', 'F', 'i', 'n', 'i', 's', 'h', 'e', 'd', '.' };

  int32_T i;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  b0 = false;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0);

  /*  this determines the time step at which the solution is given */
  /*  max iteration */
  time = 0.0;
  iter = 0;
  while (iter < 400) {
    covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 1);
    st.site = &emlrtRSI;
    time_trajj(time, posd, veld);
    st.site = &b_emlrtRSI;
    local(time, posc, velc);
    if (!b0) {
      for (i = 0; i < 3; i++) {
        dv0[i] = 0.0;
        dv1[i] = 0.0;
        dv2[i] = 0.0;
        dv3[i] = 0.0;
      }

      for (i = 0; i < 2; i++) {
        dv4[i] = 0.0;
      }

      b0 = true;
    }

    st.site = &c_emlrtRSI;
    controller(posc, velc, dv0, dv1, posd, veld, dv2, dv3, dv4, &F_traj[iter], M);
    des_z[iter] = posd[2];
    curr_z[iter] = posc[2];
    for (i = 0; i < 3; i++) {
      M_traj[iter + 400 * i] = M[i];
    }

    ttraj[iter] = time;
    time += 0.01;
    iter++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 2);
  st.site = &cb_emlrtRSI;
  figure(&st, emlrt_marshallOut(1.0), &d_emlrtMCI);
  st.site = &ab_emlrtRSI;
  plot(&st, b_emlrt_marshallOut(ttraj), b_emlrt_marshallOut(des_z),
       c_emlrt_marshallOut('r'), &emlrtMCI);
  st.site = &bb_emlrtRSI;
  hold(&st, d_emlrt_marshallOut(&st, cv0), &e_emlrtMCI);
  st.site = &y_emlrtRSI;
  plot(&st, b_emlrt_marshallOut(ttraj), b_emlrt_marshallOut(curr_z),
       c_emlrt_marshallOut('b'), &b_emlrtMCI);
  st.site = &x_emlrtRSI;
  ylabel(&st, c_emlrt_marshallOut('z'), &f_emlrtMCI);
  st.site = &w_emlrtRSI;
  disp(&st, e_emlrt_marshallOut(&st, cv1), &c_emlrtMCI);
}

/* End of code generation (run_sim.c) */
