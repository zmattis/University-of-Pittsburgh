
// OpenMP ThreadPrivate Example


// Inclusions
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

// Global Variable for ThreadPrivate
int num = 0;


// Main
int main( int argc, char** argv ) {

  // Disable Dynamic Threads
  omp_set_dynamic( 0 );

  // Set Num as ThreadPrivate
  #pragma omp threadprivate( num )

  int thread = 0;  // Thread Number
  num        = 6;  // Change Master Num Value

  printf( "First Parallel Region\n" );
  #pragma omp parallel private( thread ) copyin( num )
  {
    thread = omp_get_thread_num( );
    num    = thread * thread + num;
    printf( "  Thread %d - Value %d\n", thread, num );
  }

  printf( "Master Thread\n" );

  printf( "Second Parallel Region\n" );
  #pragma omp parallel private( thread )
  {
    thread = omp_get_thread_num( );
    printf( "  Thread %d - Value Remains %d\n", thread, num );
  }

  return 0;

}



// End threadprivate.c  - EWG SDG
