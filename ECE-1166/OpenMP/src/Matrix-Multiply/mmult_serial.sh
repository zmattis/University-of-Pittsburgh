#!/bin/bash

module purge
module load intel/2017.3.196

## To add OpenMP support, you pass to the Intel compiler the -qopenmp option.
## The -o <desired_executable_name> instructs the compiler to name the binary
## instead using the default a.out

icc mmult_serial.c -o mmult_serial.x
#sbatch mmult_serial.slurm
