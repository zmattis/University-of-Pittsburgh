
// OpenMP Basic Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char** argv ) {

  #pragma omp parallel
  {

    printf( "Hello from a random thread!\n" );

  }

  return 0;

}



// End example.c  - EWG SDG
