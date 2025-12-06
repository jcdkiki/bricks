/* Siconos is a program dedicated to modeling, simulation and control
 * of non smooth dynamical systems.
 *
 * Copyright 2024 INRIA.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *ßfailed
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "solvers.h"

#include <assert.h>  // for assert
#include <float.h>   // for DBL_EPSILON
#include <math.h>    // for fabs
#include <stdio.h>   // for printf, NULL
#include <stdlib.h>  // for malloc, free
#include <linalg.h>
#include <string.h>

struct LinearComplementarityProblem {
    int size;
    double *M;
    double *q;
};

void lcp_pivot_covering_vector(LinearComplementarityProblem* problem, double* u,
                               double* s, int* info,
                               double* cov_vec);

void lcp_pivot(LinearComplementarityProblem* problem, double* u, double* s, int* info) {
  lcp_pivot_covering_vector(problem, u, s, info, NULL);
}

#define MAX_ITER 1000

struct SolverOptions {
  int iter_done;
};

void init_M_lemke(double* mat, double* M, unsigned int dim,
                  unsigned int size_x, double* q, double* d) {
  /* construction of mat matrix such that
   * mat = [ q | Id | -d | -M ] with d_i = 1 if i < size_x
   */

  /* We need to init only the part corresponding to Id */
  memset(&mat[dim], 0, sizeof(double) * dim * dim);

  /*  Copy M but mat[dim+2:, :] = -M */
  for (unsigned int i = 0; i < dim; ++i)
    for (unsigned int j = 0; j < dim; ++j)
      mat[i + dim * (j + dim + 2)] = -M[dim * j + i];  // Siconos is in column major

  for (unsigned int i = 0; i < dim; ++i) {
    mat[i] = q[i];
    mat[i + dim * (i + 1)] = 1.0;
  }

  /** Add covering vector */
  if (d != NULL)
    for (unsigned int i = 0; i < size_x; ++i) mat[i + dim * (dim + 1)] = d[i];
  else
    for (unsigned int i = 0; i < size_x; ++i) mat[i + dim * (dim + 1)] = -1.0;
  for (unsigned int i = size_x; i < dim; ++i) mat[i + dim * (dim + 1)] = 0.0;
}

int pivot_init_lemke(double* mat, unsigned int dim) {
  int block = 0;
  double zb, dblock;
  double z0 = mat[0];

  for (unsigned int i = 1; i < dim; ++i) {
    zb = mat[i];
    if (zb < z0) {
      z0 = zb;
      block = i;
    } else if (zb == z0) {
      for (unsigned int j = 1; j <= dim; ++j) {
        dblock = mat[block + j * dim] - mat[i + j * dim];
        if (dblock < 0.)
          break;
        else if (dblock > 0.) {
          block = i;
          break;
        }
      }
    }
  }
  /* XXX check that */
  return z0 < 0.0 ? block : -1;
}

#define PIVOT_PATHSEARCH_SUCCESS -2

void do_pivot_driftless(double* mat, unsigned int dim, unsigned int dim2, unsigned int block,
                        unsigned int drive) {
  if (fabs(mat[block + drive * dim]) < DBL_EPSILON) {
    printf("do_pivot_driftless :: pivot value too small %e; q[block] = %e; theta = %e\n",
           mat[block + drive * dim], mat[block], mat[block] / mat[block + drive * dim]);
  }
  double pivot_inv = 1.0 / mat[block + drive * dim];
  unsigned ncols = dim * dim2;

  /* Update column mat[block, :] */
  mat[block + drive * dim] = 1.; /* nm_rs = 1 */
  /* nm_rj = m_rj/m_rs */
  for (unsigned int i = 0; i < drive; ++i) mat[block + i * dim] *= pivot_inv;
  for (unsigned int i = drive + 1; i < dim2; ++i) mat[block + i * dim] *= pivot_inv;

  /* Update other columns*/
  for (unsigned int i = 0; i < block; ++i) {
    double tmp = mat[i + drive * dim];
    /* nm_ij = m_ij + (m_ir/m_rs)m_rj = m_ij - m_is*nm_rj */
    for (unsigned int j = 0; j < ncols; j += dim) mat[i + j] -= tmp * mat[block + j];
  }
  for (unsigned int i = block + 1; i < dim; ++i) {
    double tmp = mat[i + drive * dim];
    /* nm_ij = m_ij + (m_ir/m_rs)m_rj = m_ij - m_is*nm_rj */
    for (unsigned int j = 0; j < ncols; j += dim) mat[i + j] -= tmp * mat[block + j];
  }
}

int pivot_selection_lemke(double* mat, unsigned dim, unsigned drive, unsigned aux_indx) {
  int block = -1;
  double candidate_pivot, candidate_ratio, dblock;
  double ratio = INFINITY;
  for (unsigned i = 0; i < dim; ++i) {
    candidate_pivot = mat[i + drive * dim];
    if (candidate_pivot > 0.) {
      candidate_ratio = mat[i] / candidate_pivot;
      if (candidate_ratio > ratio)
        continue;
      else if (candidate_ratio < ratio) {
        ratio = candidate_ratio;
        block = i;
      } else {
        if (block == (int)aux_indx || i == aux_indx) {
          /* We want the auxilliary variable to exit before any othe.
           * see CPS p. 279 and example 4.4.16 */
          block = aux_indx;
        } else {
          double current_pivot = mat[block + drive * dim];
          for (unsigned j = 1; j <= dim; ++j) {
            //assert(block >= 0 && "ratio_selection_lemke: block < 0");
            dblock = mat[block + j * dim] * candidate_pivot - mat[i + j * dim] * current_pivot;
            
            if (dblock < 0.)
              break;
            else if (dblock > 0.) {
              block = i;
              break;
            }
          }
        }
      }
    }
  }
  return block;
}

void lcp_pivot_covering_vector(LinearComplementarityProblem* problem, double* u,
                               double* s, int* info,
                               double* cov_vec) {
  double* M = problem->M;
  unsigned int dim = problem->size;
  unsigned int dim2;

  unsigned drive = dim + 1;
  int bck_drive = -1;
  int block = -1;
  unsigned has_sol = 0;
  unsigned nb_iter = 0;
  unsigned leaving = 0;
  unsigned itermax = MAX_ITER;

  double pivot;
  double tmp;
  int* basis;
  int basis_init = 0; /* 0 if basis was not initialized, 1 otherwise*/
  unsigned t_indx = 0;
  unsigned aux_indx = 0;
  double* t_stack = NULL;
  double* mat;

  *info = 0;

  /* Allocation */
  dim2 = 2 * (dim + 1);

  int stack_size = 0;
  // with pathsearch we need a stack of the basis
  basis = (int*)malloc(dim * sizeof(int));
  mat = (double*)malloc((stack_size + dim * dim2) * sizeof(double));
  t_stack = &mat[dim * dim2];

  init_M_lemke(mat, M, dim, dim, problem->q, cov_vec);

  if (!basis_init) {
    for (unsigned int i = 0; i < dim; ++i) basis[i] = i + 1;
  }

  /* Looking for pivot */
  block = pivot_init_lemke(mat, dim);

  if (block < 0) {
    if (block == -1) {
      /** exit, the solution is at hand with the current basis */
      printf("Trivial solution\n");
      goto exit_lcp_pivot;
    } else if (block == PIVOT_PATHSEARCH_SUCCESS) {
      printf("lcp_pivot :: path search successful ! t_indx = %d\n", t_indx);
      bck_drive = t_indx; /* XXX correct ? */
      t_stack[nb_iter % stack_size] = 1.0;
      double pivot = 1.0; /* force value of pivot to avoid numerical issues */
      for (unsigned int i = 0; i < dim; ++i) mat[i] -= mat[i + drive * dim] * pivot;
      *info = 0;
      goto exit_lcp_pivot;
    }
  }

  aux_indx = block;

  /* Pivot < mu , drive >  or < drive, drive > */

  pivot = mat[block + drive * dim];

  /* update matrix */
  do_pivot_driftless(mat, dim, dim2, block, drive);

  /* Update the basis */
  /** one basic u is leaving and mu enters the basis */
  leaving = basis[block];
  basis[block] = drive;

  while (nb_iter < itermax && !has_sol) {
    ++nb_iter;

    /* Start research of argmin lexico for minimum ratio test */

    /* Looking for pivot */
    if (leaving < dim + 1) {
      drive = leaving + dim + 1;
    } else if (leaving > dim + 1) {
      drive = leaving - (dim + 1);
    }
    block = pivot_selection_lemke(mat, dim, drive, aux_indx);

    if (block < 0) {
      /* We stop here: it either mean that the algorithm stops here or that there
       * is an issue with the LCP */
      if (block == -1) {
        printf(
            "The pivot column is nonpositive ! We are on ray !\n"
            "It either means that the algorithm is not able to finish or that the LCP is "
            "infeasible\n"
            "Check the class of the M matrix to find out the meaning of this\n");
        goto _exit;
        break;
      }
      /* path search was successful, t = 1, we need to update the value of the
       * basic variable, but we are done here :) */
      else if (block == PIVOT_PATHSEARCH_SUCCESS) {
        printf("lcp_pivot :: path search successful ! t_indx = %d\n", t_indx);
        basis[t_indx] = drive;
        t_stack[nb_iter % stack_size] = 1.0;
        double pivot = (mat[t_indx] - 1.0) / mat[t_indx + drive * dim];
        for (unsigned int i = 0; i < dim; ++i) mat[i] -= mat[i + drive * dim] * pivot;
        mat[t_indx] = pivot;
        *info = 0;
        break;
      }
    }

    // printf("driving variable %i \n", drive);
    if (basis[block] == (int)dim + 1) {
      has_sol = 1;
    }

    /* Pivot < block , drive > */
    // printf("Pivoting %i and %i\n", block, drive);

    pivot = mat[block + drive * dim];

    /* update matrix */
    do_pivot_driftless(mat, dim, dim2, block, drive);

    /* determine leaving variable and update basis */
    /** one basic variable is leaving and the driving one enters the basis */
    leaving = basis[block];
    basis[block] = drive;
  } /* end while*/

exit_lcp_pivot:

  /* Recover solution */
  for (unsigned int i = 0; i < dim; ++i) {
    drive = basis[i];
    assert(drive > 0);
    // assert(drive != dim + 1);
    if (drive < dim + 1) {
      u[drive - 1] = 0.0;
      s[drive - 1] = mat[i];
    } else if (drive > dim + 1) {
      u[drive - dim - 2] = mat[i];
      s[drive - dim - 2] = 0.0;
    } else {
      if (nb_iter < itermax) {
        assert(bck_drive >= 0);
        u[bck_drive] = 0.0;
        s[bck_drive] = 0.0;
      }
    }
  }

  /* update info */
  /* info may already be set*/
  if (*info == 0) {
    if (has_sol)
      *info = 0;
    else
      *info = 1;
  }

_exit:

  if (*info > 0) {
    printf("No solution found !\n");
  }

  free(basis);
  free(mat);
}

void SolveLemke(Matrix *M_in, Vector *q_in, Vector *z_out)
{
    LinearComplementarityProblem lcp;
    lcp.q = (double*)q_in->data;
    lcp.size = q_in->len;
    
    double *M_transposed = (double*)malloc(sizeof(double) * M_in->rows * M_in->cols);
    for (int i = 0; i < M_in->rows; i++) {
        for (int j = 0; j < M_in->cols; j++) {
            M_transposed[j * M_in->rows + i] = MATRIX_AT(*M_in, i, j);
        }
    }

    lcp.M = M_transposed;

    double *u = (double*)calloc(lcp.size, sizeof(double));

    int info;
    lcp_pivot(&lcp, (double*)z_out->data, u, &info);
    free(u);
    free(M_transposed);
}
