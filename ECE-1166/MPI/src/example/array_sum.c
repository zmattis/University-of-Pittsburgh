#include <stdio.h>
#include <stdlib.h>
#include <math.h>                       // To use the sqrt function
#include <string.h>                     // To use the memcpy function
#include <unistd.h>
#include <mpi.h>                        // Main parallel library

#define MAIN_NODE	0
#define ARR_SIZE	1024

/*---------------------------------------------------------------------
 * Function       :   main
 * Purpose        : Main program function, sets up MPI, calls all three functions
 * In args
 *  argv         : array of command line arguments
 *  argc         : number of command line arguments
 */
int main(int argc, char **argv)
{
	/* MPI variables */
	MPI_Init(&argc, &argv);

	// myproc = the rank of the current process
	// nprocs = the size of the communicator (total number of processing nodes)
	// i = loop variable (reused for terseness)
	int myproc = 0, nprocs = 0, i = 0;

	MPI_Comm_rank(MPI_COMM_WORLD, &myproc); // get the rank of the process
	MPI_Comm_size(MPI_COMM_WORLD, &nprocs); // get the size of the communicator

	// variable to hold the sum of the array
	int total_sum = 0;
	// will be different sizes between nodes, therefore uninitialized here
	int *my_awesome_array;
	// slice size for the array, handles arbitrary numbers of nodes
	int division_size = floor(ARR_SIZE / nprocs);
	if (myproc == nprocs - 1) {
		// send whats left to the last process
		division_size = ARR_SIZE - (division_size * (nprocs - 1));
	}

	if (myproc == MAIN_NODE) {
		// declare full array on main node
		my_awesome_array = malloc(ARR_SIZE * sizeof(int));
		for (i = 0; i < ARR_SIZE; i++)
		{
			my_awesome_array[i] = i;
		}
	} else {
		// declare partial array on all other nodes
		my_awesome_array = malloc(division_size * sizeof(int));
	}

	// scatter the array to the masses
	MPI_Scatter(my_awesome_array, ARR_SIZE, MPI_INT, my_awesome_array,
	    division_size, MPI_INT, MAIN_NODE, MPI_COMM_WORLD);

	int partial_sum = 0;
	for (i = 0; i < division_size; i++)
	{
		partial_sum += my_awesome_array[i];
	}

	// add all processors contents together
	const int elem_size = 1;
	MPI_Reduce(&partial_sum, &total_sum, elem_size, MPI_INT, MPI_SUM, MAIN_NODE, MPI_COMM_WORLD);
	// implicit barrier with reduction call, otherwise we would need MPI_Barrier here

	if (myproc == MAIN_NODE) {
		// print out sum on main node
		fprintf(stderr, "Total sum = %d\n", total_sum);
	}
	// no memory leaks, make sure to free memory before you call MPI_Finalize
	free(my_awesome_array);

	MPI_Finalize();
	return (0);
}