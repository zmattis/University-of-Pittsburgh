#!/bin/bash
module purge
module load gcc/5.4.0
module load openmpi/3.0.0

## Once you load the GNU compilers, you will have access to the mpicc wrapper for building c MPI codes.
## You can add multiple source files after the mpicc.  The -o <desired_executable_name> option instructs
## the compiler to name your binary instead of the default a.out.

mpicc hello_world.c -o hello_world.X

