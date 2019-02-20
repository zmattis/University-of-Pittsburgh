# Conway's Game of Life

Zachary M. Mattis

### Description

OpenMP C implementation of Conway's Game of Life. This implementation utilizes Row-Major Order memory layout to organize a 2-D array of char's in contiguous memory. The partitioning of work is divided via data decomposition utilizing a "BLOCK, *" 2-D scheme. Additonally, this implementation utilizes a mesh grid, wherein each node only considers its physical neighbors and NOT corresponding opposite edge nodes for those nodes located on the edge of the grid.

### Source

1. conway.c   - game of life serial
2. life.c     - game of life openmp
3. life_gen.c - game of life .txt input file generator
4. *.sh       - corresponding bash build script
5. *.slurm    - corresponding slurm job submission script
6. *.txt      - starting grid input file

### Rules

1. Any live cell with fewer than two live neighbors dies, as if by underpopulation.
2. Any live cell with two or three live neighbors lives on to the next generation.
3. Any live cell with more than three live neighbors dies, as if by overpopulation.
4. Any dead cell with exactly three live neighbors becomes a live cell, as if by reproduction.


### Compilation

Uniquely random Game of Life .txt configurations can be created using the generation file, specifying the number of rows and columns.

```bash
$ gcc -o life_gen.x life_gen.c
$ ./life_gen.x <row_size> <column_size>
```

The OpenMP source code can be compiled on Pitt's CRC cluster using its corresponding bash script.

```bash
$ ./life.sh
$ ./conway.sh
```

Each job can be submitted to CRC queue using its corresponding slurm file. The slurm file specifies the number of processing elements, input file name, number of iterations, and print frequency.

```bash
$ sbatch life.slurm
$ sbatch conway.slurm
```