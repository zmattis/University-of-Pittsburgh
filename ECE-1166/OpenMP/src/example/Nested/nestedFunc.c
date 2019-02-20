
// OpenMP Nested Functions Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv ) {

  omp_set_nested( 1 );   // Enable Nested Parallelism
  omp_set_dynamic( 0 );  // Disable Dynamic Threads

  int num     = 0;    // Thread Number
  int threads = 0;    // Current Threads
  int max     = 0;    // Maximum Threads

  // Master Report In
  num     = omp_get_thread_num( );   // Get Thread Number
  threads = omp_get_num_threads( );  // Get Current Number of Threads
  max     = omp_get_max_threads( );  // Get Maximum Number of Threads
  printf( "Master : Thread %d of %d (%d Max)\n\n", num, threads, max );

  // Outer Level Parallel Region - 2 Threads
  #pragma omp parallel num_threads( 8 )
  {

    num     = omp_get_thread_num( );   // Get Thread Number
    threads = omp_get_num_threads( );  // Get Current Number of Threads
    max     = omp_get_max_threads( );  // Get Maximum Number of Threads
    printf( "Outer  : Thread %d of %d (%d Max)\n\n", num, threads, max );

    // Inner Level Parallel Region - 2 Threads Each
    #pragma omp parallel num_threads( 4 )
    {

      num     = omp_get_thread_num( );   // Get Thread Number
      threads = omp_get_num_threads( );  // Get Current Number of Threads
      max     = omp_get_max_threads( );  // Get Maximum Number of Threads
      printf( "Inner  : Thread %d of %d (%d Max)\n", num, threads, max );

    }

  }

  return 0;

}



// End nestedFunc.c  - EWG SDG
