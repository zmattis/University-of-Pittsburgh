/**
 * @author Zachary M. Mattis
 * ECE 1166
 * Conway's Game of Life Generator
 * February 14, 2019
 *
 * This program generates a .txt file that is used as the starting
 * grid for Conway's Game of Life. The dimensions for the rows and
 * columns are provided via command line arguments.
 */

/* Header Files */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Macros */
#define ARG_ROW_SIZE 1
#define ARG_COL_SIZE 2

/* Global Variables */
const char* FILE_NAME = "life_grid.txt";
const char CHAR_ALIVE = 'o';
const char CHAR_DEAD = 'x';
const char CHAR_DELIM = ' ';

int main (int argc, char *argv[]) {

  // usage
  if (argc != 3) {
    printf("usage: ./life_gen.x <row_size> <column_size>");
    exit(EXIT_FAILURE);
  }

  // parse command line arguments
  int row_size = atoi(argv[ARG_ROW_SIZE]);
  int col_size = atoi(argv[ARG_COL_SIZE]);

  // create new file
  FILE *file_ptr = fopen(FILE_NAME, "w");

  // variables
  int i, j;

  // create 2-D grid, randomly generate dead/alive cells
  for (i=0; i<row_size; i++) {

      //fprintf(file_ptr, (i!=0 ? "\n" : "") );
      //printf( i!=0 ? "\n" : "" );

    for (j=0; j<col_size; j++) {
      fprintf(file_ptr, (j!=0 ? "%c" : ""), CHAR_DELIM);
      //printf( (j!=0 ? "%c" : "" ), CHAR_DELIM);
      fprintf(file_ptr, "%c", (rand() % 2 ? CHAR_ALIVE : CHAR_DEAD) );
      //printf("%c", (rand() % 2 ? CHAR_ALIVE : CHAR_DEAD) );
    }
      fprintf(file_ptr, "\n");
  }

  // finish
  fclose(file_ptr);
  return EXIT_SUCCESS;

}
