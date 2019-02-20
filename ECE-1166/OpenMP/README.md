# ECE 1166 - OpenMP

Zachary M. Mattis

### Description

Open Multi-Processing (OpenMP) C applications run on the University of Pittsburgh's Center for Research Computing (CRC) cluster, available h2p.crc.pitt.edu.

### Source

Includes code for the following problems:

*   Matrix Multiplication
*   Mandelbrot
*   Conway's Game of Life

### Compilation

Each C source code can be compiled on Pitt's CRC cluster using its corresponding bash script.

```bash
$ ./*.sh
```

Each job can be submitted to CRC queue using its corresponding slurm file.

```bash
$ sbatch *.slurm
```