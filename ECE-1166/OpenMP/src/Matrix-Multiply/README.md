# Matrix Multiplication

Zachary M. Mattis

### Description

Matrix Multiplication in C, implemented both serially and concurrently using OpenMP. To change the size of the matrices, change the macros in the *.c source code:

```c
SIZE_M, SIZE_N, SIZE_P
```

To change the number of processors, change x in the corresponding *.slurm file to the desired number of threads:

```slurm
#SBATCH --ntasks-per-node=x
...
export OMP_NUM_THREADS=x
```


### Source

1. mmult_serial.c - matrix multiplication serial
2. mmult_openmp.c - matrix multiplication openmp
4. *.sh           - corresponding bash build script
5. *.slurm        - corresponding slurm job submission script


### Compilation

The OpenMP source code can be compiled on Pitt's CRC cluster using its corresponding bash script.

```bash
$ ./mmult_serial.sh
$ ./mmult_openmp.sh
```

Each job can be submitted to CRC queue using its corresponding slurm file. The slurm file specifies the number of processing elements, input file name, number of iterations, and print frequency.

```bash
$ sbatch mmult_serial.slurm
$ sbatch mmult_openmp.slurm
```