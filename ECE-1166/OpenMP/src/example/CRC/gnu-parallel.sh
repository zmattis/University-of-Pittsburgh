#!/bin/bash

module purge
module load gcc/5.4.0

## To add OpenMP support, you pass to the GNU compiler the -fopenmp option.
## The -o <desired_executable_name> instructs the compiler to name the binary 
## instead using the default a.out

g++ -fopenmp hello_world_id.c -o hello_world_id.X

