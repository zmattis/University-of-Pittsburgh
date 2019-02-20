
// OpenMP Shared/Private/Default Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

int main( int argc, char* argv[ ] ) {

  int id = 0;
  int i  = 0;
  int m  = 0;
  int x  = 2;

  #pragma omp parallel private( id, i ) shared( m ) \
                       default( shared )
  {

    id = omp_get_thread_num( );
    if( id == 0 ) {
      i = 3;
      m = 17;
      x++;
    }
    printf( "Thread %d: %d %d %d\n", id, i, m, x );

  }

  return 0;

}



// End shared_private_default.c  - EWG SDG
