
// OpenMP Non-Nested Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv ) {

  // Not turning on nesting or turning off dynamic threads.

  // Outer Level Parallel Region - 2 Threads
  #pragma omp parallel num_threads( 2 )
  {

    printf( "Outer Level - You will see this twice.\n" );

    // Inner Level Parallel Region - 2 Threads Each
    #pragma omp parallel num_threads( 2 )
    {

      printf( "Inner Level - You will NOT see this four times!  Important steps are not complete!\n" );

    }

  }

  return 0;

}



// End nonNested.c  - EWG SDG
