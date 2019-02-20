/**
 * @author Zachary M. Mattis
 * ECE 1166
 * OpenMP Conway's Game of Life
 * February 18, 2019
 *
 * Purpose:
 *      The game of life! A file is provided by the user,
 *      a square grid of o's and x's, symbolizing an initial
 *      grid of either a live or a dead cell. The rules
 *      of the game are that a live cell having either 2 or
 *      3 neighbors lives on, more or less than that it dies.
 *      A dead cell with exactly 3 neighbors lives on to the next
 *      generation. The grid is a torus, so cells on the edge
 *      of the grid are adjacent to their opposite cells.
 *
 * Compile:
 *    gcc -Wall -o life.x life.c -lm -fopenmp
 * Usage:
 *    ./life <cores> <infile name> <# of iterations> <print frequency>
 * Input:
 *    cores - number of cores to use during execution
 *    infile name - the initial map name as a file, elements, o= alive, x=dead
 *    iterations - total number of steps to run the simulation
 *    print frequency - the view update frequency (must be >= iterations)
 *
 * Output:
 *    The grid at the final step multiple of the state given by d in the update frequency
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <omp.h>
#include <unistd.h>

// 1 = alive, 0 = dead
typedef unsigned char cell;
#define ALIVE		'o'
#define DEAD		'x'

#define KRED		"\x1B[31m"
#define KGRN		"\x1B[36m"
#define RESET		"\x1B[0m"

#define IS_ALIVE(x)	(x == ALIVE)
#define IS_DEAD(x)	(x == DEAD)

#define ARG_CORES          1
#define ARG_FILE_NAME      2
#define ARG_NUM_ITERATIONS 3
#define ARG_PRINT_FREQ     4

#define MILLISECONDS 1000

/*---------------------------------------------------------------------
 * Function       :   getRowsAndCols
 * Purpose        :   Reads the input file given by the user and automatically
 *                    calculates how many rows and columns there are
 * In args
 * filename       :   name of input file
 * In/Out args
 * rows, columns  :   # rows and # columns integer pointers
 */
void getRowsAndCols(const char *filename, int *rows, int *cols);

/*---------------------------------------------------------------------
 * Function       :   initializeGrid
 * Purpose        :   Reads the input file given by the user and automatically
 *                    calculates how many rows and columns there are
 * In args
 * filename       :   name of input file
 * In/Out args
 * grid           :   grid of cells, preallocated
 */
void initializeGrid(cell *grid, const char *filename);

/**
 * [printGrid prints game of life grid with color in bash]
 * @param grid [grid of cells]
 * @param rows [number of rows in the grid]
 * @param cols [number of columns in the grid]
 * @param step [step number that is being printed]
 * @param outputFile [file to print to]
 */
void printGrid(cell *grid, const int rows, const int cols, const int step, FILE *outputFile);

/*---------------------------------------------------------------------
 * Function       :   getNeighborCount
 * Purpose        :   Calculates the number of neighboring cells alive
 *                    for a given cell
 * In args
 * lifeGrid       :   ptr to life grid
 * rows           :   # of rows in life grid
 * cols           :   # of cols in life grid
 * x              :   x position of current cell
 * y              :   y position of current cell
 * Return
 *                :   # of neighboring cells alive
 */
int getNeighborCount(cell *lifeGrid, const int rows, const int cols, int x, int y);

int main(int argc, char **argv)
{
  if (argc != 5) {
    fprintf(stderr, "usage: ./life <cores> <infile name> <# of iterations> <print frequency> \n");
    exit(EXIT_FAILURE);
  }

  double time_start, time_end;
  time_start = omp_get_wtime();

  cell *lifeGrid, *tempGrid, *swapPtr;      // ROW-MAJOR ORDER
  int rows = 0, cols = 0, neighborCount = 0, i = 0, j = 0, k = 0;
  const int cores = atoi(argv[ARG_CORES]);

  getRowsAndCols(argv[ARG_FILE_NAME], &rows, &cols);

  const int num_steps = atoi(argv[ARG_NUM_ITERATIONS]);
  const int print_frequency = atoi(argv[ARG_PRINT_FREQ]);

  char *outfile_name = calloc(100, sizeof(char));
  snprintf(outfile_name, 100, "%s_%s_%s_%s.out", argv[ARG_CORES], argv[ARG_FILE_NAME], argv[ARG_NUM_ITERATIONS], argv[ARG_PRINT_FREQ]);

  lifeGrid = calloc(rows * cols, sizeof(cell));   // life grid
  tempGrid = calloc(rows * cols, sizeof(cell));   // temp grid for calculations
  initializeGrid(lifeGrid, argv[ARG_FILE_NAME]);


  for ( ; i < num_steps; i++)
  {
    if (i % print_frequency == 0) {
      printGrid(lifeGrid, rows, cols, i, stdout);
    }

    // Game of Life
    #pragma omp parallel for private(j, k, neighborCount) shared (lifeGrid, tempGrid, rows, cols) default (none)
    for (j=0; j<rows; j++ ) {
      //printf("Hello x%d from thread: %d\n", j, omp_get_thread_num() );
      for (k=0; k<cols; k++) {
        neighborCount = getNeighborCount( lifeGrid, rows, cols, j, k );

        if(neighborCount == 2) tempGrid[j*cols+k] = lifeGrid[j*cols+k];
        if(neighborCount == 3) tempGrid[j*cols+k] = ALIVE;
        if(neighborCount < 2) tempGrid[j*cols+k] = DEAD;
        if(neighborCount > 3) tempGrid[j*cols+k] = DEAD;
      }
    }

    // swap grids
    swapPtr = lifeGrid;
    lifeGrid = tempGrid;
    tempGrid = swapPtr;
  }


  FILE *outfile = fopen(outfile_name, "w");
  printGrid(lifeGrid, rows, cols, i, outfile);

  // end conways game of life
  fclose(outfile);
  free(outfile_name);
  free(tempGrid);
  free(lifeGrid);

  time_end = omp_get_wtime();
  printf("Execution Time (s): %f\n", time_end-time_start );

  return (EXIT_SUCCESS);
}


void getRowsAndCols(const char *filename, int *rows, int *cols)
{
  FILE *gridfile;

  // enough room to store each cell plus space, initialized with zeros
  char *row = calloc(2, sizeof(char));
  int i = 0;

  if ((row) && (filename)) {
    // on process 0, open the file to read
    gridfile = fopen(filename, "r");

    if (!gridfile) {
      fprintf(stderr, "Something went wrong when reading %s", filename);
      exit(EXIT_FAILURE);
    }
    // read in each row until rows
    *rows = 0;
    *cols = 0;
    while (fscanf(gridfile, "%s", row) != EOF)
    {
      i += 1;
      if ((getc(gridfile) == '\n')) {
        *rows += 1;
      }
    }
  } else {
    fprintf(stderr, "Something went wrong...");
    exit(EXIT_FAILURE);
  }
  *cols = i / *rows;
  fclose(gridfile);
  free(row);
}


void initializeGrid(cell *myGrid, const char *filename)
{
  FILE *gridfile;

  // enough room to store each cell plus space, initialized with zeros
  char *row = calloc(3, sizeof(char));
  int i = 0;

  if ((row) && (myGrid) && (filename)) {
    // on process 0, open the file to read
    gridfile = fopen(filename, "r");

    if (!gridfile) {
      fprintf(stderr, "Something went wrong when reading %s", filename);
      exit(EXIT_FAILURE);
    }
    while (fscanf(gridfile, "%s", row) != EOF)
    {
      myGrid[i++] = *row;
    }
  } else {
    fprintf(stderr, "Something went wrong...");
    exit(EXIT_FAILURE);
  }
  fclose(gridfile);
  free(row);
}


void printGrid(cell *grid, const int rows, const int cols, const int step, FILE *output)
{
#pragma omp barrier
  {
    const int size = rows * cols;
    const int buffsize = 10;
    int i;

    fprintf(output, "%dx%d Grid for step %d\n", rows, cols, step);

    char *out = calloc(buffsize, sizeof(char));
    for (i = 0; i < size; i++)
    {
      if ((i > 0) && (i % cols == 0)) {
        fprintf(output, "\n");
      }
      if ((output == stdout) || (output == stderr)) {
        //strcat(out, grid[i] == DEAD ? KRED : KGRN);
      }
      strcat(out, (grid[i] == DEAD ? "x " : "o "));
      if ((output == stdout) || (output == stderr)) {
        //strcat(out, RESET);
      }
      fprintf(output, out);
      memset(out, 0, buffsize);
    }
    fprintf(output, "\n\n");
    free(out);
  }
}

int getNeighborCount(cell *lifeGrid, const int rows, const int cols, int x, int y) {

  int	i, j;
  int count = 0;

  // go around the cell
  for (i=-1; i<=1; i++) {
    for (j=-1; j<=1; j++) {

      // no middle cell
      if ( i || j ) {
        // bound checks
        if ( x+i>=0 && x+i<=rows-1 && y+j>=0 && y+j<=cols-1 ) {
          //printf( "checking life_grid[%d][%d]\n", x+i, y+j );
          if ( IS_ALIVE(lifeGrid[ (x+i)*cols+y+j ]) ) {
            count++;
          }
        }
      }
    }
  }
  //printf("count: %d\n", count );
  return count;
}
