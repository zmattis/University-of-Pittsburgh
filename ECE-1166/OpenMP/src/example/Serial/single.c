
// OpenMP Single Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int i = 0;  // Loop Iterator
  int n = 0;  // Number of Iterations

  // Parallel Region
  #pragma omp parallel    \
              shared( n ) \
              private( i )
  {

    #pragma omp for             // Parallelize For Loop
    for( i = 0; i < n; i++ ) {  // Iterate Through
      printf( "Thread %d of %d - Iteration %d\n",
              omp_get_thread_num( ),
              omp_get_max_threads( ), i           );
    }

    #pragma omp single          // Single Section - Executed by One Core
    {
      printf( "Thread %d of %d - Running Single Construct\n",
              omp_get_thread_num( ),
              omp_get_max_threads( )                          );
    }

  }

  return 0;

}



// End single.c  - EWG SDG
