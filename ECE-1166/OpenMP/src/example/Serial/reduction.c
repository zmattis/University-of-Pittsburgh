
// OpenMP Reduction Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int  *a      = malloc( 25 * sizeof( int ) );  // Array of Values
  int   i      = 0;                             // Loop Index
  int   n      = 25;                            // Number of Loop Iterations
  int   sum    = 0;                             // Sum for All Cores
  int   thread = 0;                             // Thread Number
  float start  = 0.0;                           // Start Time
  float end    = 0.0;                           // End Time
  float time   = 0.0;                           // Elapsed Time

  // Fill Array with Values 1 to 25
  for( i = 0; i < n; i++ ) {
    a[i] = i + 1;
  }

  start = omp_get_wtime( );

  // Parallel Region
  #pragma omp parallel for shared( n, a ) private( thread ) reduction( + : sum )
    // Note the Added For and No Braces
    // Share Number of Iterations, Array, and Total Sum
    // Keep Thread Number and Local Sum Private
    // Reduce the Sum Variable Using Addition (+)
  for( i = 0; i < n; i++ ) {  // Iterate Through Number of Iterations
    sum += a[i];              // Add Array Values to Local Sum
  }
  // Reduce Operation Takes Place at End of Parallel Region

  end  = omp_get_wtime( );
  time = end - start;
  printf( "Total Sum at End:  %d\nTime:  %0.9lf\n", sum, time );
  free( a );

  return 0;

}



// End reduction.c  - EWG SDG
