
/*=======================
    M A N D E L B R O T
  =======================*/

// Implementation Based on Rosetta Code Example
// 1) Draws Mandelbrot set for Fc(z)=z*z +c using
//    Mandelbrot algorithm (boolean escape time).
// 2) Technique of creating ppm file is  based on
//    the code of Claudio Rocchini.  http://en.
//    wikipedia.org/wiki/Image:Color_complex_plot
//    .jpg.  Create 24 bit color graphic file,
//    portable pixmap file = PPM, see http://en.
//    wikipedia.org/wiki/Portable_pixmap to see
//    the file use external application (graphic
//    viewer).


// Inclusions
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <omp.h>


// Definitions
#define MAX_COLOR_COMPONENT_VALUE 255
#define I_MAX                     200
#define CXMIN                      -1.5
#define CXMAX                       0.5
#define CYMIN			          		   -1.0
#define CYMAX				            		1.0
#define ESCAPE_RADIUS_2             4.0
// Image Structure Definition
typedef struct {
  unsigned char color[3];
} image;


// Function Prototypes
void instructions( int argc, char** argv );


// Main
int main ( int argc, char **argv ) {

  instructions( argc, argv );  // Display Usage Instructions If Wrong Arguments

  double time_start, time_end;
  time_start = omp_get_wtime();

           char   *filename     = argv[1];          // Parse Input Arguments
  unsigned int     iXMax        = atoi( argv[2] );  // Generated Image Width
  unsigned int     iYMax        = iXMax;            // Generated Image Height
  unsigned int     display      = atoi( argv[3] );  // Argument to Display Debug Text
  unsigned int     i            = 0;                // Iteration Number
  unsigned int     iX           = 0;                // Screen (Integer) X Coordinate
  unsigned int     iY           = 0;                // Screen (Integer) Y Coordinate
  unsigned int     thisPixelNum = 0;                // Iterator for Tracking Pixel Number
           double  cX           = 0.0;              // World (Double) X Coordinate
           double  cY           = 0.0;              // World (Double) Y Coordinate
           double  zX           = 0.0;              // Z = Zx + Zy * i; Z0 = 0
           double  zY           = 0.0;              // (see just above)
           double  zX2          = 0.0;              // Square of Zx
           double  zY2          = 0.0;              // Square of Zy
           char   *comment      = "# ";             // Dynamic File Header Comment

  // Intro Text and Setup
  if( display ) {
    printf( "\n   = = =  Mandelbrot Set Generator  = = =   \n\n" );
  }
  unsigned int    size        = iXMax * iYMax;              // Determination of Size
           double pixelWidth  = ( CXMAX - CXMIN ) / iXMax;  // Determination of Pixel Width/
           double pixelHeight = ( CYMAX - CYMIN ) / iYMax;  //   Height from Window/Size
           image *fractal     = malloc( size * sizeof( *fractal ) );  // Allocate Storage for Image

  // Compute Fractal Image
  if( display ) {
    printf( "Generating Mandelbrot Set...\n" );
  }
  #pragma omp parallel for private(iY, iX, cY, cX, zY, zX, zY2, zX2, i, thisPixelNum) shared (iYMax, iXMax, pixelHeight, pixelWidth, fractal) default (none) schedule(guided)
  for( iY = 0; iY < iYMax; iY++ ) {    // Iterate Through Image Rows
    cY = CYMIN + iY * pixelHeight;
    if( fabs( cY ) < ( pixelHeight / 2 ) ) {
      cY = 0.0;                        // Main Antenna
    }
    for( iX = 0; iX < iXMax; iX++ ) {  // Iterate Through Image Columns
      cX = CXMIN + iX * pixelWidth;
      zX  = 0.0;                       // Initial Value of Orbit - Critical Point Z = 0
      zY  = 0.0;
      zX2 = zX * zX;
      zY2 = zY * zY;
      for( i = 0; ( i < I_MAX ) && ( ( zX2 + zY2 ) < ESCAPE_RADIUS_2 ); i++ ) {
        zY  = 2 * zX * zY + cY;
        zX  = zX2 - zY2 + cX;
        zX2 = zX * zX;
        zY2 = zY * zY;
      };
      // Save Pixel Color
      thisPixelNum                     = iY * iYMax + iX;  // Where is this pixel in the image?
      if( i == I_MAX ) {                                   // Color for Interior of Mandelbrot Set
        fractal[thisPixelNum].color[0] = 37;               // Red
        fractal[thisPixelNum].color[1] = 37;               // Green
        fractal[thisPixelNum].color[2] = 37;               // Blue
      } else {                                             // Color for Exterior of Mandelbrot Set
        fractal[thisPixelNum].color[0] = 0;                // Red
        fractal[thisPixelNum].color[1] = 0;                // Green
        fractal[thisPixelNum].color[2] = 255;              // Blue
      }  // End If
    }    // End iX For
  }      // End iY For

  // Image File Write Phase
  if( display ) {
    printf( "Writing File Out...\n" );
  }
  // Create New File - give it a name and open it in binary mode.
  FILE *filePtr = fopen( filename, "wb" );  // b - Binary Mode
  // Write ASCII Header to the File
  fprintf( filePtr, "P6\n %s\n %d\n %d\n %d\n", comment, iXMax, iYMax, MAX_COLOR_COMPONENT_VALUE );
  // Image File Write Out - must be done serially.
  for( iY = 0; iY < iYMax; iY++ ) {
    for( iX = 0; iX < iXMax; iX++ ) {
      thisPixelNum = iY * iYMax + iX;                        // Set Dereference Pixel Location
      fwrite( fractal[thisPixelNum].color, 1, 3, filePtr );  // Write Pixel Color to File
    }
  }

  // Final Tasks
  fclose( filePtr );
  free( fractal );
  if( display ) {
    printf( "Operation Complete!\n\n" );
  }

  time_end = omp_get_wtime();
  printf("Execution Time (s): %f\n", time_end-time_start );

  return EXIT_SUCCESS;
}


// Function Implementations

// Instructions - display usage instructions if argument count incorrent.
void instructions( int argc, char** argv ) {
  if( argc != 4 ) {
    printf( "\nUsage: %s <output> <x/y> <display>\n", argv[0] );
    printf( "  Output  - a .ppm image to output with the fractal.\n" );
    printf( "  X/Y     - width and height of image in pixels.\n" );
    printf( "  Display - 1 displays debug text, 0 just displays time values for raw data tables.\n\n" );
    exit( EXIT_FAILURE );
  }
}


// End mandelbrot.c  - EWG SDG
