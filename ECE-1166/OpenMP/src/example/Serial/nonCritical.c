
// OpenMP Non-Critical Example


// Inclusion
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int *a        = malloc( 25 * sizeof( int ) );  // Array of Values
  int  i        = 0;                             // Loop Iterator
  int  n        = 25;                            // Number of Iterations
  int  localSum = 0;                             // Private Local Sum for Each Core
  int  totalSum = 0;                             // Shared Total Sum for All Cores
  int  thread   = 0;                             // Thread Number

  // Fill Array with Values 1 to 25
  for( i = 0; i < n; i++ ) {
    a[i] = i + 1;
  }

  // Parallel Region
  #pragma omp parallel                 \
              shared( n, a, totalSum ) \
              private( thread, localSum )
  {

    thread   = omp_get_thread_num( );  // Get Thread Number
    localSum = 0;                      // Preset Local Sum to Zero

    #pragma omp for                    // Parallelize the Following For Loop
    for( i = 0; i < n; i++ ) {         // Iterate Through
      localSum += a[i];                // Accumulate Values in Local Sum
    }

    totalSum += localSum;              // All Cores Accumulate Values
    printf( "Thread %d has local sum %d and adds to total sum %d.\n",
            thread, localSum, totalSum );
    // Bad - No Critcal Section
    //   There is nothing keeping each thread from accessing this variable
    //   whenever it wishes.  A race condition occurs.  While "totalSum +=
    //   localSum" looks like a swift, in-place operation, there is a load
    //   of the value that is currently there, a modification, and then a
    //   store back into that shared memory space.  If one core loads the
    //   value while another core is changing it, before it has a chance
    //   to store it back, the first cores addition will not be seen by
    //   second core, and will then be overwritten by the second core's
    //   store.

  }

  printf( "Total sum at end is %d.\n", totalSum );
  free( a );

  return 0;

}



// End nonCritical.c  - EWG SDG
