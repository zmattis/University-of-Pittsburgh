
// OpenMP FirstPrivate Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>


// Main
int main( int argc, char** argv ) {

  int thread = 0;  // Thread Number
  int num    = 3;  // Number

  printf( "Master Thread\n  Num Value = %d\n", num );
  printf( "Parallel Region\n" );
  #pragma omp parallel private( thread ) firstprivate( num )
  {
    #pragma omp master
    {
      printf( "  Num Value = %d\n", num );
    }
    thread = omp_get_thread_num( );
    num    = thread * thread + num;
    printf( "  Thread %d - Value %d\n", thread, num );
  }

  printf( "Master Thread\n  Num Value = %d\n", num );

  return 0;

}



// End firstprivate.c  - EWG SDG
