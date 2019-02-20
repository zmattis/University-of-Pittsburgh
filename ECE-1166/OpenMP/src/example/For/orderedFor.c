
// OpenMP Ordered For Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int    i      = 0;    // Loop Iterator
  int    n      = 0;    // Number of Iterations
  double start  = 0.0;  // Start Time
  double middle = 0.0;  // Middle Time
  double end    = 0.0;  // End Time
  double for1   = 0.0;  // For Loop 1 Time
  double for2   = 0.0;  // For Loop 2 Time
  double total  = 0.0;  // Total Time

  // Parallel Region
  #pragma omp parallel    \
              shared( n ) \
              private( i )
  {

    start = omp_get_wtime( );   // Get Start Time

    #pragma omp for ordered     // Parallelize Ordered For Loop
    for( i = 0; i < n; i++ ) {  // Iterate Through
      #pragma omp ordered       // Ordered Applies Here
      printf( "Thread %d of %d - Iteration %d\n",
              omp_get_thread_num( ),
              omp_get_max_threads( ), i           );
    }

    middle = omp_get_wtime( );  // Get Middle Time

    #pragma omp for ordered     // Parallelize Ordered For Loop
    for( i = 0; i < n; i++ ) {  // Iterate Through
      #pragma omp ordered       // Ordered Applies Here
      printf( "Thread %d of %d - Iteration %d\n",
              omp_get_thread_num( ),
              omp_get_max_threads( ), i           );
    }

    end = omp_get_wtime( );     // Get End Time

  }

  // Calculate Time
  for1  = middle - start;
  for2  = end    - middle;
  total = end    - start;

  // Display Time
  printf( "For Loop 1:  %0.9lf\n", for1 );
  printf( "For Loop 2:  %0.9lf\n", for2 );
  printf( "Total Time:  %0.9lf\n", total );

  return 0;

}



// End orderedFor.c  - EWG SDG
