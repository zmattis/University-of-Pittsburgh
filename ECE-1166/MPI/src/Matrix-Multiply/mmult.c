/**
 * @author Zachary M. Mattis
 * ECE 1166
 * MPI Matrix Multiplication
 * January 30, 2019
 *
 * This program is a matrix multiply program for use over a distrubted
 * computing network, capable of multiplying an (M X N) * (N x P) matrix sizes.
 *
 * This program was successfully compiled on Pitt's CRC cluster using the
 * compilation bash script, mmult.sh. The job was submited by the following
 * command: $ sbatch mmult.slurm
 */

/* Header Files */
#include "mpi.h"
#include <stdio.h>
#include <stdlib.h>

/* Macros */
#define SIZE_M 100                 // rows A
#define SIZE_N 100                 // cols A, rows B
#define SIZE_P 100                 // cols B
#define MASTER 0                   // master processor

#define MASTER_TAG 1         // message from master
#define WORKER_TAG 2         // message from worker
#define MICROSECONDS 1000000

/* Function Declaration */
void printMatrix( double (*matrix)[SIZE_P], int x, int y );    // static 2D array requires that second level of pointers be constpointer to static array


int main (int argc, char *argv[]) {
    int	num_tasks,             // number of tasks in partition
      world_rank,              // task identifier
      num_workers,             // number of worker tasks
      src,                     // message source
      dest,                    // message destination
      m_type,                  // message type
      rows,                    // rows of matrix A sent to each worker
      row_diff, extra, offset, // determine rows sent to each worker
      i, j, k;                 // misc

  double	matrix_a[SIZE_M][SIZE_N],
    matrix_b[SIZE_N][SIZE_P],
    matrix_c[SIZE_M][SIZE_P],
    t0, t1;                  // timing variables;
  MPI_Status status;

  MPI_Init(&argc,&argv);
  t0 = MPI_Wtime();
  MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &num_tasks);

  num_workers = num_tasks-1;


  // MASTER
  if (world_rank == MASTER)
  {
      // init
      for (i=0; i<SIZE_M; i++)
        for (j=0; j<SIZE_N; j++)
            matrix_a[i][j]= i+j;
      for (i=0; i<SIZE_N; i++)
        for (j=0; j<SIZE_P; j++)
            matrix_b[i][j]= i*j;

      /* Send matrix data to the worker tasks */
      row_diff = SIZE_M/num_workers;
      extra = SIZE_M%num_workers;
      offset = 0;
      m_type = MASTER_TAG;
      for (dest=1; dest<=num_workers; dest++)
      {
        rows = (dest <= extra) ? row_diff+1 : row_diff;
        printf("Sending %d rows to task %d offset=%d\n",rows,dest,offset);
        MPI_Send(&offset, 1, MPI_INT, dest, m_type, MPI_COMM_WORLD);
        MPI_Send(&rows, 1, MPI_INT, dest, m_type, MPI_COMM_WORLD);
        MPI_Send(&matrix_a[offset][0], rows*SIZE_N, MPI_DOUBLE, dest, m_type, MPI_COMM_WORLD);
        MPI_Send(&matrix_b, SIZE_N*SIZE_P, MPI_DOUBLE, dest, m_type, MPI_COMM_WORLD);
        offset = offset + rows;
      }

      /* Receive results from worker tasks */
      m_type = WORKER_TAG;
      for (i=1; i<=num_workers; i++)
      {
        src = i;
        MPI_Recv(&offset, 1, MPI_INT, src, m_type, MPI_COMM_WORLD, &status);
        MPI_Recv(&rows, 1, MPI_INT, src, m_type, MPI_COMM_WORLD, &status);
        MPI_Recv(&matrix_c[offset][0], rows*SIZE_P, MPI_DOUBLE, src, m_type, MPI_COMM_WORLD, &status);
        printf("Received results from task %d\n",src);
      }

      t1 = MPI_Wtime();
      printf("Execution time (us) for (%d x %d) * (%d x %d) on %d procs = %f\n\n", SIZE_M, SIZE_N, SIZE_N, SIZE_P, num_tasks, (t1-t0)*MICROSECONDS);

      // print results
      //printMatrix(matrix_c, SIZE_M, SIZE_P);
  }

  // WORKER
  if (world_rank != MASTER)
  {

      /* Receive input from master */
      m_type = MASTER_TAG;
      MPI_Recv(&offset, 1, MPI_INT, MASTER, m_type, MPI_COMM_WORLD, &status);
      MPI_Recv(&rows, 1, MPI_INT, MASTER, m_type, MPI_COMM_WORLD, &status);
      MPI_Recv(&matrix_a, rows*SIZE_N, MPI_DOUBLE, MASTER, m_type, MPI_COMM_WORLD, &status);
      MPI_Recv(&matrix_b, SIZE_N*SIZE_P, MPI_DOUBLE, MASTER, m_type, MPI_COMM_WORLD, &status);

      for (k=0; k<SIZE_P; k++)
        for (i=0; i<rows; i++)
        {
            matrix_c[i][k] = 0.0;
            for (j=0; j<SIZE_N; j++)
              matrix_c[i][k] = matrix_c[i][k] + matrix_a[i][j] * matrix_b[j][k];
        }

      /* Send calculated matrix data to master */
      m_type = WORKER_TAG;
      MPI_Send(&offset, 1, MPI_INT, MASTER, m_type, MPI_COMM_WORLD);
      MPI_Send(&rows, 1, MPI_INT, MASTER, m_type, MPI_COMM_WORLD);
      MPI_Send(&matrix_c, rows*SIZE_P, MPI_DOUBLE, MASTER, m_type, MPI_COMM_WORLD);
  }
  MPI_Finalize();
}

/**
 * Prints an x by y matrix to stdout
 * @param {double **} matrix   Ptr to matrix
 * @param {int} row            Num rows
 * @param {int} col            Num cols
 */
void printMatrix( double (*matrix)[SIZE_P], int row, int col ) {
      int i, j;

      printf("-------------------------------------------------------\n");
      printf("Result Matrix:\n");
      for (i=0; i<row; i++) {
        printf("\n");
        for (j=0; j<col; j++) {
          printf("%6.2f   ", matrix[i][j]);
        }
      }
      printf("\n-------------------------------------------------------\n");
      printf ("Done.\n");

}