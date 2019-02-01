#!/bin/bash
module purge
module load intel/2017.3.196

## Once you load the Intel compilers, you will have access to the mpiicc wrapper for building c MPI codes.
## You can add multiple source files after the mpiicc.  The -o <desired_executable_name> option instructs
## the compiler to name your binary instead of the default a.out.

mpiicc mmult.c -o mmult.x
#sbatch mmult.slurm
