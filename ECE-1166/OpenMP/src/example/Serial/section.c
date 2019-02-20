
// OpenMP Section Example

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

void functionA( void ) {
  int a = omp_get_thread_num( );
  printf( "This is function A!  Executed by Core %d\n", a );
}

void functionB( void ) {
  int b = omp_get_thread_num( );
  printf( "This is function B!  Executed by Core %d\n", b );
}

int main( int argc, char** argv ) {

  int i = 0;

  #pragma omp parallel private( i )
  {

    i = omp_get_thread_num( );

    #pragma omp sections
    {

      #pragma omp section
      functionA( );

      #pragma omp section
      functionB( );

    }

    printf( "All cores again.  This one is %d.\n", i );

  }

  return 0;

}



// End section.c  - EWG SDG
