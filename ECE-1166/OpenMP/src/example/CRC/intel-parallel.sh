#!/bin/bash

module purge
module load intel/2017.3.196

## To add OpenMP support, you pass to the Intel compiler the -qopenmp option.
## The -o <desired_executable_name> instructs the compiler to name the binary
## instead using the default a.out

icc -qopenmp hello_world_id.c -o hello_world_id.x

