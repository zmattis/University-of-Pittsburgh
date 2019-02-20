/**
 * @author Zachary M. Mattis
 * ECE 1166
 * OpenMP Matrix Multiplication
 * February 19, 2019
 *
 * This program is a matrix multiply program,
 * capable of multiplying an (M X N) * (N x P) matrix sizes.
 * The matrices of this program are stored in memory
 * in row-major order.
 *
 * This program was successfully compiled on Pitt's CRC cluster using the
 * compilation bash script, mmult.sh. The job was submited by the following
 * command: $ sbatch mmult.slurm
 */

/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include <time.h>

/* Macros */
#define SIZE_M 10                 // rows A
#define SIZE_N 10                 // cols A, rows B
#define SIZE_P 10                 // cols B

/* Function Declaration */
void printMatrix( double *matrix, int row, int col );


int main (int argc, char *argv[]) {

  // matrix memory
  double *matrix_a, *matrix_b, *matrix_c;

  matrix_a = calloc( SIZE_M * SIZE_N, sizeof(double) );
  matrix_b = calloc( SIZE_N * SIZE_P, sizeof(double) );
  matrix_c = calloc( SIZE_M * SIZE_P, sizeof(double) );

  // variables
  int i, j, k;

  // init
  #pragma omp parallel for private(i, j) shared (matrix_a) default (none)
  for (i=0; i<SIZE_M; i++)
    for (j=0; j<SIZE_N; j++)
      matrix_a[i*SIZE_N+j]= i+j;

  #pragma omp parallel for private(i, j) shared (matrix_b) default (none)
  for (i=0; i<SIZE_N; i++)
    for (j=0; j<SIZE_P; j++)
      matrix_b[i*SIZE_P+j]= i*j;

  // timing information
  double time_start, time_end;
  time_start = omp_get_wtime();
  printf("time_start: %f\n", time_start);

  // multiply
  #pragma omp parallel for private(i, j, k) shared (matrix_a, matrix_b, matrix_c) default (none)
  for (k=0; k<SIZE_P; k++) {
    for (i=0; i<SIZE_M; i++) {
      matrix_c[i*SIZE_P+k] = 0.0;
      for (j=0; j<SIZE_N; j++) {
        matrix_c[i*SIZE_P+k] += matrix_a[i*SIZE_N+j] * matrix_b[j*SIZE_P+k];
      }
    }
  }

  // timing
  time_end = omp_get_wtime();
  printf("time_end: %f\n", time_end);
  printf( "Execution time (s) for (%d x %d) * (%d x %d) = %f\n\n", SIZE_M, SIZE_N, SIZE_N, SIZE_P, time_end-time_start );

  // print results
  //printMatrix( matrix_a, SIZE_M, SIZE_N );
  //printMatrix( matrix_b, SIZE_N, SIZE_P );
  //printMatrix( matrix_c, SIZE_M, SIZE_P);

  // finalize
  free(matrix_a);
  free(matrix_b);
  free(matrix_c);

  return EXIT_SUCCESS;

}

/**
 * Prints an x by y matrix to stdout
 * @param {double **} matrix   Ptr to matrix
 * @param {int} row            Num rows
 * @param {int} col            Num cols
 */
void printMatrix( double *matrix, int row, int col ) {
      int i, j;

      printf("-------------------------------------------------------\n");
      printf("Result Matrix:\n");
      for (i=0; i<row; i++) {
        printf("\n");
        for (j=0; j<col; j++) {
          printf("%6.2f   ", matrix[i*col+j]);
        }
      }
      printf("\n-------------------------------------------------------\n");
      printf ("Done.\n");

}