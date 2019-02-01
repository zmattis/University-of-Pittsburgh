/**
 * @author Zachary M. Mattis
 * ECE 1166
 * MPI Asynchronous Communication
 * January 30, 2019
 *
 * This program is a communication test between two nodes
 * of a parallel computing architecture utilizing asynchronous functions.

 * This program was successfully compiled on Pitt's CRC cluster using the
 * compilation bash script, compile.sh. The job was submited by the following
 * command: $ sbatch ping_pong_async.slurm
 */

/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <mpi.h>

/* Macros */
#define PING	0
#define PONG  1

#define ARG_MSSG_SIZE 1
#define NO_TAG 0
#define MICROSECONDS 1000000

int main(int argc, char **argv) {

  // MPI variables
	MPI_Init(&argc, &argv);

  if (argc != 2) {
    printf("Usage:\n\t\tmpirun -np 2 ./ping_pong_async.x <data_size>");
    exit(EXIT_FAILURE);
  }

  // Get number of packets to send from command line, simple iterator
  int mssg_size = atoi(argv[ARG_MSSG_SIZE]);
  int i;

  // Get processor MPI rank
  int world_rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

  // Request status
  MPI_Request req_send, req_recv;

  // data buffer, allocate number of packets
  double *mssg;
  mssg = (double *)malloc(mssg_size * sizeof(double));


  // PING
  if (world_rank == PING) {

    // timing Information
    double t0, t1;
    double total_time, avg_time;

    // get current time, send to pong
    t0 = MPI_Wtime();

    // create data packet, time repeated x times
    for(i=0; i<mssg_size; i++) {
      mssg[i] = t0;
    }

    MPI_Isend( mssg, mssg_size, MPI_DOUBLE, PONG, NO_TAG, MPI_COMM_WORLD, &req_send );
    //printf("Ping sent %f x%d to Pong\n", *mssg, mssg_size);

    // recv data from pong, wait, record time
    MPI_Irecv( mssg, mssg_size, MPI_DOUBLE, PONG, NO_TAG, MPI_COMM_WORLD, &req_recv );
    MPI_Wait( &req_recv, MPI_STATUS_IGNORE );

    t1 = MPI_Wtime();
    //printf("Ping received %f x%d from Pong\n", *mssg, mssg_size);

    // check data recv from pong
    if (*mssg != t0) {
      printf("Message data corrupted! Process exiting\n\n");
      exit(EXIT_FAILURE);
    }

    total_time = t1 - t0;
    avg_time = total_time/2;

    printf("-------- ASYNCHRONOUS MPI COMMUNICATION --------\n");
    printf("Bytes sent: %d\n", mssg_size*sizeof(double));
    printf("Round trip time (us): %f\n", total_time*MICROSECONDS);
    printf("Average time (us): %f\n\n\n", avg_time*MICROSECONDS);

  }

  // PONG
  if (world_rank == PONG) {

    // Receive time from PING, wait, forward back
    MPI_Irecv( mssg, mssg_size, MPI_DOUBLE, PING, NO_TAG, MPI_COMM_WORLD, &req_recv );
    MPI_Wait( &req_recv, MPI_STATUS_IGNORE );
    //printf("Pong received %f x%d from Ping\n", *mssg, mssg_size);

    MPI_Isend( mssg, mssg_size, MPI_DOUBLE, PING, NO_TAG, MPI_COMM_WORLD, &req_send );
    MPI_Wait( &req_send, MPI_STATUS_IGNORE );
    //printf("Pong sent %f x%d to Pong\n", *mssg, mssg_size);

  }

  // finished
  free(mssg);
  MPI_Finalize();
  return EXIT_SUCCESS;
}
