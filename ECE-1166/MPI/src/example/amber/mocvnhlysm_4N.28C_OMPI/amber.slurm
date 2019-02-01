#!/bin/bash
#SBATCH --nodes=4
#SBATCH --ntasks-per-node=28
#SBATCH --cluster=mpi
##SBATCH --partition=opa
#SBATCH --partition=opa-high-mem
#SBATCH --time=1:00:00
#SBATCH --job-name=mocv

# Load Modules
module purge
module load gcc/5.4.0
module load openmpi/3.0.0
module load amber/18_gcc-5.4.0

# Run over Omni-Path fabric
export I_MPI_FABRICS_LIST="tmi"
export I_MPI_FALLBACK=0

export MKL_HOME=$MKLROOT

ulimit -s unlimited

# Amber input files and output name
INP=md.in
TOP=mocvnhlysm.top
CRD=mocvnhlysm.crd
OUT=mocvnhlysm

# Executable
SANDER=pmemd.MPI

# Launch MPI using mpirun
mpirun -n $SLURM_NTASKS \
          $SANDER  -O     -i   $INP   -p   $TOP   -c   $CRD   -r   $OUT.rst \
                          -o   $OUT.out   -e   $OUT.ene   -v   $OUT.vel   -inf $OUT.nfo   -x   $OUT.mdcrd

# Launch MPI using srun (this also works)
#srun --mpi=pmi2 \
#          $SANDER  -O     -i   $INP   -p   $TOP   -c   $CRD   -r   $OUT.rst \
#                          -o   $OUT.out   -e   $OUT.ene   -v   $OUT.vel   -inf $OUT.nfo   -x   $OUT.mdcrd 
