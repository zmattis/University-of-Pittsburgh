
/*=======================
    C Program Template
    Evan William Gretok
       Month D, YEAR
  =======================*/


// Inclusions
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <omp.h>


// Definitions
#define DEBUG 0
#define SIZE  100000000


// Main
int main( int argc, char *argv[ ] ) {

  // Seed Your Random Number Generators
  srand( time( NULL ) );

  // Loop Iterator
  int i = 0;

  // Initialize Vector Memory Spaces
  double *mainVector = malloc( SIZE * sizeof( double ) );
  double *divVector  = malloc( SIZE * sizeof( double ) );  
  double *solVector  = malloc( SIZE * sizeof( double ) );

  // Fill Vectors
  double x = rand( );
  printf( "%lf\n", x );    
  for( i = 0; i < SIZE; i++ ) {
    mainVector[i] = ( rand( ) % 10000000 ) * 0.01;
    divVector[i]  = ( rand( ) % 1000 ) * 0.01;
    solVector[i]  = 0;
  }

  // DEBUG - Display Vectors
  if( DEBUG ) {
    for( i = 0; i < SIZE; i++ ) {
      printf( "  %5.2lf %1.2lf\n", mainVector[i], divVector[i] );
    }
  }

  // Perform Processing
  double start   = omp_get_wtime( );
  #pragma omp parallel for shared( mainVector, divVector, solVector ) \
                           private( i ) schedule( static, 10 ) num_threads( 4 )
  for( i = 0; i < SIZE; i++ ) {
    solVector[i] = mainVector[i] / divVector[i];
  }
  double end     = omp_get_wtime( );

  // DEBUG - Display Output
  if( DEBUG ) {
    for( i = 0; i < SIZE; i++ ) {
      printf( "  %1.2lf\n", solVector[i] );
    }
  }
  
  // Display Timing
  double solTime = end - start;
  printf( "Vector Division Complete - %lf Seconds\n", solTime );

  return 0;

}



// End .c  - EWG SDG
