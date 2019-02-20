
// OpenMP Unnamed Critical Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int  *a        = malloc( 25 * sizeof( int ) );  // Array of Values
  int   i        = 0;                             // Loop Iterator
  int   n        = 25;                            // Number of Iteratins
  int   localSum = 0;                             // Private Local Sum for Each Core
  int   totalSum = 0;                             // Shared Total Sum for All Cores
  int   thread   = 0;                             // Thread Number
  float start    = 0.0;                           // Start Time
  float end      = 0.0;                           // End Time
  float time     = 0.0;                           // Elapsed Time

  // Fill Array with Values 1 to 25
  for( i = 0; i < n; i++ ) {
    a[i] = i + 1;
  }

  // Parallel Region
  #pragma omp parallel shared( n, a, totalSum ) private( thread, localSum )
  // Share Number of Iterations, Array, and the Total Sum
  // Keep the Thread Number and Local Sum Private
  {

    thread   = omp_get_thread_num( );  // Get the Thread Number
    localSum = 0;                      // Preset Local Sum to Zero

    #pragma omp for                    // Parallelize the Next For
    for( i = 0; i < n; i++ ) {
      localSum += a[i];                // Accumulate Array Values into Local Sum
    }

    start = omp_get_wtime( );
    #pragma omp critical               // Unnamed Critical Region - blocks both.
    {
      totalSum += localSum;            // Accumulate Local Sum Values into Total Sum
      printf( "Thread %d has local sum %d and adds to total sum %d.\n",
              thread, localSum, totalSum );
    }
    #pragma omp critical               // Unnamed Critical Region - blocks both.
    {
      printf( "I'm an unnamed critical region.\n" );
    }
    end = omp_get_wtime( );
  }

  time = end - start;
  printf( "Total sum at end is %d.\nTime:  %0.9lf\n", totalSum, time );
  free( a );

  return 0;

}



// End unnamedCritical.c  - EWG SDG
