
/*=======================
    OpenMP Test
    Evan William Gretok
    February 5, 2019
  =======================*/


// Inclusions
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <omp.h>


// Main
int main( int argc, char **argv ) {

  if( argc != 2 ) {
    printf( "ERROR: must pass in a number of threads to use!\n" );
    return 7;
  }
  int threadArg  = atoi( argv[1] );
  int thisThread = 0;
  int allThreads = 0;

  #pragma omp parallel num_threads( threadArg )
  {
    thisThread   = omp_get_thread_num( );
    allThreads   = omp_get_max_threads( );
    printf( "Hello, world!  From thread %d of %d, with love.\n", thisThread, allThreads );
  }

  return 0;

}



// End OMPTest.c  - EWG SDG
