
// OpenMP Nested Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv ) {

  omp_set_nested( 1 );   // Enable Nested Parallelism
  omp_set_dynamic( 0 );  // Disable Dynamic Threads

  // Outer Level Parallel Region - 2 Threads
  #pragma omp parallel num_threads( 2 )
  {

    printf( "Outer Level - You will see this twice.\n" );

    // Inner Level Parallel Region - 2 Threads Each
    #pragma omp parallel num_threads( 2 )
    {

      printf( "Inner Level - You will see this four times!\n" );

    }

  }

  return 0;

}



// End nested.c  - EWG SDG
