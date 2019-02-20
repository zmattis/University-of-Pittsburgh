// OpenMP Master Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv ) {

  int num_threads = 0;  // Number of Threads
  int thread_id   = 0;  // ID Number of Running Thread

  #pragma omp parallel private( num_threads, thread_id )
  {

    // Get the Thread Number
    thread_id = omp_get_thread_num( );
    printf( "Hello World from Thread %d\n", thread_id );

    // Have Master Print Total Number of Threads Used
    #pragma omp master
    {
      num_threads = omp_get_num_threads( );
      printf( "Number of Threads = %d\n", num_threads );
    }

  }

  return 0;

}



// End master.c  - EWG SDG
