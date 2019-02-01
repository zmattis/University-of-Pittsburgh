# ECE 1166 - MPI

Zachary M. Mattis

### Description

Message Passing Interface (MPI) C applications run on the University of Pittsburgh's Center for Research Computing (CRC) cluster, available h2p.crc.pitt.edu.

### Compilation

Each C source code can be compiled on Pitt's CRC cluster using its corresponding bash script.

```bash
$ ./mmult.sh
```

Each job can be submitted to CRC queue using its corresponding slurm file.

```bash
$ sbatch mmult.slurm
```

Each Ping-Pong communication architecture is under a different source code file, for ease of implementation. The Ping-Pong slurm file executes a number of different Ping-Pong tests, each with a different byte range supplied via a command line argument. For the Matrix-Multiply, the size of each matrix attribute can be set in header of the C source under the macro definitions. In order to change the number of processors, the #SBATCH --nodes=x can be updated in the .slurm file can be update to the corresponding number of processors (x).
