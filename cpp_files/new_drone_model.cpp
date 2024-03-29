/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) new_drone_model_ ## ID
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)
#define casadi_to_mex CASADI_PREFIX(to_mex)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, is_sparse, c, k, p_nrow, p_ncol;
  const casadi_int *colind, *row;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  Jc = 0;
  Ir = 0;
  if (is_sparse) {
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    casadi_int nnz;
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    nnz = sp[ncol];
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch. "
                                 "Expected %d-by-%d, got %d-by-%d instead.",
                                 nrow, ncol, p_nrow, p_ncol);
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) (static_cast<double>(x))

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, c, k;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int nnz;
#endif
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  nnz = sp[ncol];
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

#ifndef CASADI_PRINTF
#ifdef MATLAB_MEX_FILE
  #define CASADI_PRINTF mexPrintf
#else
  #define CASADI_PRINTF printf
#endif
#endif

static const casadi_int casadi_s0[20] = {16, 1, 0, 16, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[7] = {3, 1, 0, 3, 0, 1, 2};

/* X_DOT:(x0[16],wr[4],w_tilt[4],params[15],dt,f_ext[3])->(pos_dot[3],vel_dot[3],eul_dot[3],wB_dot[3],tilt_dot[4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][4] : 0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][7] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][8] : 0;
  a3=cos(a2);
  a4=(a1*a3);
  a5=arg[3]? arg[3][0] : 0;
  a4=(a4/a5);
  a6=arg[3]? arg[3][1] : 0;
  a7=arg[3]? arg[3][7] : 0;
  a8=sin(a7);
  a9=arg[0]? arg[0][12] : 0;
  a10=sin(a9);
  a8=(a8*a10);
  a11=(a6*a8);
  a12=(a4*a11);
  a13=sin(a2);
  a1=(a1*a13);
  a1=(a1/a5);
  a14=cos(a7);
  a14=(a14*a10);
  a10=(a6*a14);
  a15=(a1*a10);
  a12=(a12+a15);
  a15=sin(a0);
  a15=(a15/a5);
  a9=cos(a9);
  a16=(a6*a9);
  a17=(a15*a16);
  a12=(a12+a17);
  a17=arg[1]? arg[1][0] : 0;
  a12=(a12*a17);
  a18=arg[3]? arg[3][8] : 0;
  a19=sin(a18);
  a20=arg[0]? arg[0][13] : 0;
  a21=sin(a20);
  a19=(a19*a21);
  a22=(a6*a19);
  a23=(a4*a22);
  a24=cos(a18);
  a24=(a24*a21);
  a21=(a6*a24);
  a25=(a1*a21);
  a23=(a23+a25);
  a20=cos(a20);
  a25=(a6*a20);
  a26=(a15*a25);
  a23=(a23+a26);
  a26=arg[1]? arg[1][1] : 0;
  a23=(a23*a26);
  a12=(a12+a23);
  a23=arg[3]? arg[3][9] : 0;
  a27=sin(a23);
  a28=arg[0]? arg[0][14] : 0;
  a29=sin(a28);
  a27=(a27*a29);
  a30=(a6*a27);
  a31=(a4*a30);
  a32=cos(a23);
  a32=(a32*a29);
  a29=(a6*a32);
  a33=(a1*a29);
  a31=(a31+a33);
  a28=cos(a28);
  a33=(a6*a28);
  a34=(a15*a33);
  a31=(a31+a34);
  a34=arg[1]? arg[1][2] : 0;
  a31=(a31*a34);
  a12=(a12+a31);
  a31=arg[3]? arg[3][10] : 0;
  a35=sin(a31);
  a36=arg[0]? arg[0][15] : 0;
  a37=sin(a36);
  a35=(a35*a37);
  a38=(a6*a35);
  a4=(a4*a38);
  a39=cos(a31);
  a39=(a39*a37);
  a37=(a6*a39);
  a1=(a1*a37);
  a4=(a4+a1);
  a36=cos(a36);
  a1=(a6*a36);
  a15=(a15*a1);
  a4=(a4+a15);
  a15=arg[1]? arg[1][3] : 0;
  a4=(a4*a15);
  a12=(a12+a4);
  a4=arg[5]? arg[5][0] : 0;
  a12=(a12+a4);
  if (res[1]!=0) res[1][0]=a12;
  a12=arg[0]? arg[0][6] : 0;
  a4=sin(a12);
  a40=sin(a0);
  a41=(a4*a40);
  a42=(a41*a3);
  a43=cos(a12);
  a44=sin(a2);
  a45=(a43*a44);
  a42=(a42+a45);
  a42=(a42/a5);
  a45=(a42*a11);
  a2=cos(a2);
  a43=(a43*a2);
  a41=(a41*a13);
  a43=(a43-a41);
  a43=(a43/a5);
  a41=(a43*a10);
  a45=(a45-a41);
  a41=cos(a0);
  a4=(a4*a41);
  a4=(a4/a5);
  a46=(a4*a16);
  a45=(a45-a46);
  a45=(a45*a17);
  a46=(a42*a22);
  a47=(a43*a21);
  a46=(a46-a47);
  a47=(a4*a25);
  a46=(a46-a47);
  a46=(a46*a26);
  a45=(a45+a46);
  a46=(a42*a30);
  a47=(a43*a29);
  a46=(a46-a47);
  a47=(a4*a33);
  a46=(a46-a47);
  a46=(a46*a34);
  a45=(a45+a46);
  a42=(a42*a38);
  a43=(a43*a37);
  a42=(a42-a43);
  a4=(a4*a1);
  a42=(a42-a4);
  a42=(a42*a15);
  a45=(a45+a42);
  a42=arg[5]? arg[5][1] : 0;
  a45=(a45+a42);
  if (res[1]!=0) res[1][1]=a45;
  a45=-9.8100000000000005e+00;
  a42=sin(a12);
  a44=(a42*a44);
  a4=cos(a12);
  a40=(a4*a40);
  a3=(a40*a3);
  a44=(a44-a3);
  a44=(a44/a5);
  a11=(a44*a11);
  a40=(a40*a13);
  a42=(a42*a2);
  a40=(a40+a42);
  a40=(a40/a5);
  a10=(a40*a10);
  a11=(a11-a10);
  a4=(a4*a41);
  a4=(a4/a5);
  a16=(a4*a16);
  a11=(a11+a16);
  a11=(a11*a17);
  a22=(a44*a22);
  a21=(a40*a21);
  a22=(a22-a21);
  a25=(a4*a25);
  a22=(a22+a25);
  a22=(a22*a26);
  a11=(a11+a22);
  a30=(a44*a30);
  a29=(a40*a29);
  a30=(a30-a29);
  a33=(a4*a33);
  a30=(a30+a33);
  a30=(a30*a34);
  a11=(a11+a30);
  a44=(a44*a38);
  a40=(a40*a37);
  a44=(a44-a40);
  a4=(a4*a1);
  a44=(a44+a4);
  a44=(a44*a15);
  a11=(a11+a44);
  a45=(a45+a11);
  a11=arg[5]? arg[5][2] : 0;
  a45=(a45+a11);
  if (res[1]!=0) res[1][2]=a45;
  a45=cos(a12);
  a11=cos(a0);
  a44=cos(a12);
  a11=(a11*a44);
  a44=(a45*a11);
  a4=sin(a12);
  a1=casadi_sq(a4);
  a44=(a44-a1);
  a1=(a45*a11);
  a40=casadi_sq(a4);
  a1=(a1-a40);
  a44=(a44/a1);
  a40=arg[0]? arg[0][9] : 0;
  a44=(a44*a40);
  a40=sin(a12);
  a0=sin(a0);
  a12=cos(a12);
  a0=(a0*a12);
  a12=(a40*a0);
  a12=(a12/a1);
  a37=arg[0]? arg[0][10] : 0;
  a12=(a12*a37);
  a0=(a45*a0);
  a0=(a0/a1);
  a38=arg[0]? arg[0][11] : 0;
  a0=(a0*a38);
  a12=(a12+a0);
  a44=(a44-a12);
  if (res[2]!=0) res[2][0]=a44;
  a11=(a11/a1);
  a11=(a11*a37);
  a4=(a4/a1);
  a4=(a4*a38);
  a11=(a11+a4);
  if (res[2]!=0) res[2][1]=a11;
  a40=(a40/a1);
  a40=(a40*a37);
  a45=(a45/a1);
  a45=(a45*a38);
  a40=(a40+a45);
  if (res[2]!=0) res[2][2]=a40;
  a40=sin(a7);
  a45=arg[3]? arg[3][6] : 0;
  a40=(a40*a45);
  a38=(a6*a40);
  a38=(a38*a9);
  a1=arg[3]? arg[3][2] : 0;
  a37=(a1*a8);
  a38=(a38-a37);
  a37=arg[3]? arg[3][3] : 0;
  a38=(a38/a37);
  a38=(a38*a17);
  a11=sin(a18);
  a11=(a11*a45);
  a4=(a6*a11);
  a4=(a4*a20);
  a44=(a1*a19);
  a4=(a4+a44);
  a4=(a4/a37);
  a4=(a4*a26);
  a38=(a38+a4);
  a4=sin(a23);
  a4=(a4*a45);
  a44=(a6*a4);
  a44=(a44*a28);
  a12=(a1*a27);
  a44=(a44-a12);
  a44=(a44/a37);
  a44=(a44*a34);
  a38=(a38+a44);
  a44=sin(a31);
  a44=(a44*a45);
  a12=(a6*a44);
  a12=(a12*a36);
  a0=(a1*a35);
  a12=(a12+a0);
  a12=(a12/a37);
  a12=(a12*a15);
  a38=(a38+a12);
  if (res[3]!=0) res[3][0]=a38;
  a38=(a1*a14);
  a7=cos(a7);
  a7=(a7*a45);
  a12=(a6*a7);
  a12=(a12*a9);
  a38=(a38-a12);
  a12=arg[3]? arg[3][4] : 0;
  a38=(a38/a12);
  a38=(a38*a17);
  a18=cos(a18);
  a18=(a18*a45);
  a37=(a6*a18);
  a37=(a37*a20);
  a0=(a1*a24);
  a37=(a37+a0);
  a37=(a37/a12);
  a37=(a37*a26);
  a38=(a38-a37);
  a37=(a1*a32);
  a23=cos(a23);
  a23=(a23*a45);
  a0=(a6*a23);
  a0=(a0*a28);
  a37=(a37-a0);
  a37=(a37/a12);
  a37=(a37*a34);
  a38=(a38+a37);
  a31=cos(a31);
  a31=(a31*a45);
  a45=(a6*a31);
  a45=(a45*a36);
  a37=(a1*a39);
  a45=(a45+a37);
  a45=(a45/a12);
  a45=(a45*a15);
  a38=(a38-a45);
  if (res[3]!=0) res[3][1]=a38;
  a20=(a1*a20);
  a11=(a6*a11);
  a11=(a11*a19);
  a18=(a6*a18);
  a18=(a18*a24);
  a11=(a11+a18);
  a20=(a20-a11);
  a11=arg[3]? arg[3][5] : 0;
  a20=(a20/a11);
  a20=(a20*a26);
  a40=(a6*a40);
  a40=(a40*a8);
  a7=(a6*a7);
  a7=(a7*a14);
  a40=(a40+a7);
  a9=(a1*a9);
  a40=(a40+a9);
  a40=(a40/a11);
  a40=(a40*a17);
  a20=(a20-a40);
  a4=(a6*a4);
  a4=(a4*a27);
  a23=(a6*a23);
  a23=(a23*a32);
  a4=(a4+a23);
  a28=(a1*a28);
  a4=(a4+a28);
  a4=(a4/a11);
  a4=(a4*a34);
  a20=(a20-a4);
  a1=(a1*a36);
  a44=(a6*a44);
  a44=(a44*a35);
  a6=(a6*a31);
  a6=(a6*a39);
  a44=(a44+a6);
  a1=(a1-a44);
  a1=(a1/a11);
  a1=(a1*a15);
  a20=(a20+a1);
  if (res[3]!=0) res[3][2]=a20;
  a20=1.;
  a1=arg[3]? arg[3][11] : 0;
  a1=(a20-a1);
  a15=arg[2]? arg[2][0] : 0;
  a1=(a1*a15);
  if (res[4]!=0) res[4][0]=a1;
  a1=arg[3]? arg[3][12] : 0;
  a1=(a20-a1);
  a15=arg[2]? arg[2][1] : 0;
  a1=(a1*a15);
  if (res[4]!=0) res[4][1]=a1;
  a1=arg[3]? arg[3][13] : 0;
  a1=(a20-a1);
  a15=arg[2]? arg[2][2] : 0;
  a1=(a1*a15);
  if (res[4]!=0) res[4][2]=a1;
  a1=arg[3]? arg[3][14] : 0;
  a20=(a20-a1);
  a1=arg[2]? arg[2][3] : 0;
  a20=(a20*a1);
  if (res[4]!=0) res[4][3]=a20;
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int X_DOT(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

extern "C" CASADI_SYMBOL_EXPORT int X_DOT_alloc_mem(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT int X_DOT_init_mem(int mem) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void X_DOT_free_mem(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT int X_DOT_checkout(void) {
  return 0;
}

extern "C" CASADI_SYMBOL_EXPORT void X_DOT_release(int mem) {
}

extern "C" CASADI_SYMBOL_EXPORT void X_DOT_incref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT void X_DOT_decref(void) {
}

extern "C" CASADI_SYMBOL_EXPORT casadi_int X_DOT_n_in(void) { return 6;}

extern "C" CASADI_SYMBOL_EXPORT casadi_int X_DOT_n_out(void) { return 5;}

extern "C" CASADI_SYMBOL_EXPORT casadi_real X_DOT_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* X_DOT_name_in(casadi_int i){
  switch (i) {
    case 0: return "x0";
    case 1: return "wr";
    case 2: return "w_tilt";
    case 3: return "params";
    case 4: return "dt";
    case 5: return "f_ext";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const char* X_DOT_name_out(casadi_int i){
  switch (i) {
    case 0: return "pos_dot";
    case 1: return "vel_dot";
    case 2: return "eul_dot";
    case 3: return "wB_dot";
    case 4: return "tilt_dot";
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* X_DOT_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s3;
    case 5: return casadi_s4;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT const casadi_int* X_DOT_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s4;
    case 2: return casadi_s4;
    case 3: return casadi_s4;
    case 4: return casadi_s1;
    default: return 0;
  }
}

extern "C" CASADI_SYMBOL_EXPORT int X_DOT_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_X_DOT(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  casadi_real w[107];
  casadi_int *iw = 0;
  const casadi_real* arg[6] = {0};
  casadi_real* res[5] = {0};
  if (argc>6) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"X_DOT\" failed. Too many input arguments (%d, max 6)", argc);
  if (resc>5) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"X_DOT\" failed. Too many output arguments (%d, max 5)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+59);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+16, casadi_s1, w+59);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+20, casadi_s1, w+59);
  if (--argc>=0) arg[3] = casadi_from_mex(argv[3], w+24, casadi_s2, w+59);
  if (--argc>=0) arg[4] = casadi_from_mex(argv[4], w+39, casadi_s3, w+59);
  if (--argc>=0) arg[5] = casadi_from_mex(argv[5], w+40, casadi_s4, w+59);
  --resc;
  res[0] = w+43;
  if (--resc>=0) res[1] = w+46;
  if (--resc>=0) res[2] = w+49;
  if (--resc>=0) res[3] = w+52;
  if (--resc>=0) res[4] = w+55;
  i = X_DOT(arg, res, iw, w+59, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"X_DOT\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s4, res[0]);
  if (res[1]) resv[1] = casadi_to_mex(casadi_s4, res[1]);
  if (res[2]) resv[2] = casadi_to_mex(casadi_s4, res[2]);
  if (res[3]) resv[3] = casadi_to_mex(casadi_s4, res[3]);
  if (res[4]) resv[4] = casadi_to_mex(casadi_s1, res[4]);
}
#endif

casadi_int main_X_DOT(casadi_int argc, char* argv[]) {
  casadi_int j;
  casadi_real* a;
  const casadi_real* r;
  casadi_int flag;
  casadi_int *iw = 0;
  casadi_real w[107];
  const casadi_real* arg[6];
  casadi_real* res[5];
  arg[0] = w+0;
  arg[1] = w+16;
  arg[2] = w+20;
  arg[3] = w+24;
  arg[4] = w+39;
  arg[5] = w+40;
  res[0] = w+43;
  res[1] = w+46;
  res[2] = w+49;
  res[3] = w+52;
  res[4] = w+55;
  a = w;
  for (j=0; j<43; ++j) if (scanf("%lg", a++)<=0) return 2;
  flag = X_DOT(arg, res, iw, w+59, 0);
  if (flag) return flag;
  r = w+43;
  for (j=0; j<16; ++j) CASADI_PRINTF("%g ", *r++);
  CASADI_PRINTF("\n");
  return 0;
}


#ifdef MATLAB_MEX_FILE
extern "C"
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[6];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_X_DOT(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "X_DOT")==0) {
    mex_X_DOT(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'X_DOT'");
}
#endif
int main(int argc, char* argv[]) {
  if (argc<2) {
    /* name error */
  } else if (strcmp(argv[1], "X_DOT")==0) {
    return main_X_DOT(argc-2, argv+2);
  }
  fprintf(stderr, "First input should be a command string. Possible values: 'X_DOT'\nNote: you may use function.generate_input to create a command string.\n");
  return 1;
}
